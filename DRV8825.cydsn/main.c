/* ========================================================================
 * Quad-stepper carrier + CY8CKIT-059: two DRV8825 spinning continuously
 * Target: CY8CKIT-059 (PSoC 5LP) plugged into the quad carrier PCB
 *
 * NO TopDesign components required, everything is register-level (same
 * approach as the zeta project): GPIO via per-pin PC registers, timing from
 * the free-running Cortex-M3 SysTick counter at BUS_CLK.
 *
 * WHAT IT DOES: both motors ramp up once and then turn forever, without
 * stopping or reversing. Set DIR_1 / DIR_2 below to pick each direction.
 * The DRV8825 FAULT line is polled and reported - see FAULT below.
 *
 * -------------------- PIN MAP (from the carrier schematic) -----------------
 * From the carrier schematic J1 (the CY8CKIT-059 right-hand header, pin 1 =
 * P2_0 at the bottom, pin 24 = P1_7, 25 = GND, 26 = VDDIO). Corrected
 * 2026-07-19 by the board owner: driver 2's dir2/step2/Sleep2 wires run from
 * J1 pins 12/13/14 = P12[4]/P12[3]/P12[2], while RST2 runs from P1[2].
 * P1[0]/P1[1] (the SWD programming pins) are deliberately skipped.
 *
 *   header:      EN     M0     M1     M2     RST    SLP     STEP    DIR
 *   Driver 1:    P2[7]  P2[6]  P2[5]  P2[4]  P2[3]  P2[2]   P2[1]   P2[0]
 *   Driver 2:    P1[7]  P1[6]  P1[5]  P1[4]  P1[2]  P12[2]  P12[3]  P12[4]
 *
 * (schematic net names: EN1/M1_0/M1_1/M1_2/RST1/Sleep1/step1/dir1 and the
 * matching *2 set - "M1_2" means driver 1's M2 input, not P1[2].)
 * P1[0]/P1[1] are unconnected on the carrier, so SWD keeps working and no
 * MLOGIC_DEBUG tricks are needed. Confirmed by multimeter: with the
 * old (wrong) map, the socket's SLP pin floated at ~4.5 V on the module
 * pull-up and STEP/DIR sat at 0 V because P12[3]/P12[4] were never driven.
 *
 * FAULT: the schematic labels the FAULT pin of BOTH driver modules "FAULT1"
 * and brings it to P12[5]. DRV8825 FAULT is open-drain and active low, so
 * wiring two of them to one line is a normal wired-OR - which also means an
 * asserted FAULT does NOT say WHICH driver tripped. There is no pull-up
 * drawn on the carrier, so P12[5] is configured here with the PSoC's
 * internal pull-up (the trick the zeta project uses for its limit switch).
 * A low reading means a driver is in overcurrent, thermal shutdown, or
 * undervoltage lockout - the last of which is what you see with VMOT absent.
 *
 * P2[1] (driver 1 STEP) is also the on-board blue LED. STEP idles HIGH
 * between the brief low notches, so the LED sits lit while driver 1 is
 * stepping - a free "the firmware is commanding steps" indicator. It
 * reports what is COMMANDED, not what the shaft does: a stalled motor
 * still lights it. (P2[2], driver 1 SLP, is the kit's user button SW1:
 * pressing it would put driver 1 to sleep - just don't.)
 *
 * P12[6]/P12[7] carry no carrier signal, so the KitProg USB-UART is free:
 * the debug log lives on P12[7], 115200 8N1, readable on the KitProg COM
 * port. It prints a banner, the fault state, and a revolution count.
 *
 * STEP RECIPE: FULL_SPS is reached through a startup ramp, NOT commanded
 * from rest - a 1.8 deg stepper only starts reliably below roughly 1-2 rev/s,
 * so commanding 5 rev/s directly just makes it buzz in place. The ramp
 * (ACCEL_SPS2) walks the rate up from ~32 steps/s, after which the motor
 * holds cruise indefinitely. FULL_SPS defaults to a deliberately slow
 * 200 steps/s (1 rev/s) for bring-up; raise it once the motors actually turn.
 *
 * STEP pulses are a brief LOW notch (STEP_LOW_US) on an otherwise HIGH pin;
 * the DRV8825 steps on the rising edge. Short pulses matter here: the pulse
 * is a blocking delay, and with two motors sharing one loop a 500 us pulse
 * would consume an entire 1 ms step period and leave no scheduling margin.
 *
 * IF IT BUZZES BUT DOES NOT TURN, the pin map is not the suspect any more -
 * it is now schematic-confirmed. Check, in order: (1) motor phase pairing -
 * the 17L102S-LW8 is an 8-lead motor and mis-paired coils are the classic
 * cause of exactly this symptom (J6/J7 pins 1,2 = phase A, pins 3,4 = phase
 * B); (2) VREF on each driver pot (<= 0.35 V for 0.7 A/phase); (3) the FAULT
 * report in the log.
 *
 * POWER: VMOT from the J5 terminal via external supply, never from USB.
 * Never plug/unplug a motor while VMOT is powered.
 * ======================================================================== */
#include <cytypes.h>
#include <cyfitter.h>       /* BCLK__BUS_CLK__HZ                      */
#include <cydevice_trm.h>   /* CYREG_PRTx_PCy, CYREG_NVIC_SYSTICK_*   */
#include <cypins.h>
#include <CyLib.h>
#include <stdint.h>
#include <stdio.h>

/* ========================= CHOOSE WHAT THIS BUILD DOES ==================== */
#define WIRING_PROBE_MODE   (0u)  /* 1 = toggle every mapped pin at 1 Hz, one
                                     at a time, logging its name - probe the
                                     stepper headers with a multimeter to
                                     verify the PIN MAP above.               */
/* ========================================================================= */

#define USE_DEBUG_UART      (1u)      /* KitProg COM port, P12[7] is free    */
#define LOG_PERIOD_MS       (3000u)   /* revolution + fault report interval  */

/* ------------------------------ motion tuning ----------------------------- */
/* Bring-up default is slow on purpose: 1 rev/s is inside the pull-in range
 * even without the ramp, so if the motor will turn at all, it turns here.
 * Raise FULL_SPS once you have seen both shafts move.                       */
#define USTEP_DIV           (8u)      /* 1/8 microstep: smooth rotation. M
                                         pins are schematic- and probe-
                                         verified, safe to rely on.          */
#define FULL_SPS            (2000u)    /* cruise rate, full-steps/s (1 rev/s) */
#define ACCEL_SPS2          (6400u)   /* startup ramp rate, usteps/s^2       */

#define STEP_SPS            (FULL_SPS * USTEP_DIV)      /* STEP edges per s  */
#define STEPS_PER_REV       (200u * USTEP_DIV)          /* 1.8 deg motor     */
#define C_MIN               ((uint32)(BCLK__BUS_CLK__HZ / STEP_SPS))
#define LOG_CY              ((uint32)((BCLK__BUS_CLK__HZ / 1000u) * LOG_PERIOD_MS))

#define STEP_LOW_US         (100u)    /* STEP low notch (DRV8825 min 1.9 us;
                                         generous width in case anything on
                                         the carrier slows the edges)        */
#define DIR_SETUP_US        (20u)     /* DIR to first STEP edge              */

/* rotation sense depends on how each motor's coils are wired; flip either */
#define DIR_1               (1u)
#define DIR_2               (1u)

/* ------------------- register-level pins (no components) ------------------ */
typedef struct
{
    uint32 dir;     /* DRV8825 DIR                                  */
    uint32 step;    /* DRV8825 STEP (rising edge = one microstep)   */
    uint32 slp;     /* DRV8825 nSLEEP (1 = awake)                   */
    uint32 rst;     /* DRV8825 nRESET (1 = run)                     */
    uint32 m2;      /* microstep select                             */
    uint32 m1;
    uint32 m0;
    uint32 en;      /* DRV8825 nENABLE (0 = outputs on)             */
} drv_t;

/* Driver 1: schematic nets dir1/step1/Sleep1/RST1/M1_2/M1_1/M1_0/EN1 */
static const drv_t DRV1 = {
    CYREG_PRT2_PC0, CYREG_PRT2_PC1, CYREG_PRT2_PC2, CYREG_PRT2_PC3,
    CYREG_PRT2_PC4, CYREG_PRT2_PC5, CYREG_PRT2_PC6, CYREG_PRT2_PC7
};
/* Driver 2: schematic nets dir2/step2/Sleep2/RST2/M2_2/M2_1/M2_0/EN2.
 * dir2/step2/Sleep2 are on P12, NOT P1 - the carrier skips the SWD pins. */
static const drv_t DRV2 = {
    CYREG_PRT12_PC4, CYREG_PRT12_PC3, CYREG_PRT12_PC2, CYREG_PRT1_PC2,
    CYREG_PRT1_PC4, CYREG_PRT1_PC5, CYREG_PRT1_PC6, CYREG_PRT1_PC7
};

#define PIN_FAULT           CYREG_PRT12_PC5   /* FAULT1, open-drain, act low */
#define PIN_DBG_TX          CYREG_PRT12_PC7   /* P12[7] -> KitProg USB serial */
#define PIN_LED_STEP1       (DRV1.step)       /* P2[1]: blue LED = DRV1 STEP  */

#define FAULT_ACTIVE()      (CyPins_ReadPin(PIN_FAULT) == 0u)

/* ------------------------------ soft timer -------------------------------- */
/* free-running 24-bit SysTick as the cycle timebase (no interrupt), plus a
 * software-extended 32-bit elapsed counter for step scheduling (the 24-bit
 * counter wraps every ~0.7 s at 24 MHz). The 32-bit accumulator wraps every
 * ~179 s, which is why every deadline test below is a signed difference -
 * that stays correct across the wrap as long as the interval is < 2^31.     */
#define SU_NOW()            (CY_GET_REG32(CYREG_NVIC_SYSTICK_CURRENT))
#define SU_ELAPSED(t0)      (((t0) - SU_NOW()) & 0x00FFFFFFu)

#define DBG_BIT_CY          (BCLK__BUS_CLK__HZ / 115200u)

static void su_timer_start(void)
{
    CY_SET_REG32(CYREG_NVIC_SYSTICK_RELOAD, 0x00FFFFFFu);
    CY_SET_REG32(CYREG_NVIC_SYSTICK_CURRENT, 0u);
    CY_SET_REG32(CYREG_NVIC_SYSTICK_CTL, 0x05u);  /* enable, CPU clock, no IRQ */
}

static uint32 g_tb_last;
static uint32 g_tb_acc;

static void tb_reset(void)
{
    g_tb_last = SU_NOW();
    g_tb_acc  = 0u;
}

/* must be polled at least every ~0.69 s; the spin loop polls continuously */
static uint32 tb_now32(void)
{
    uint32 raw = SU_NOW();
    g_tb_acc += (g_tb_last - raw) & 0x00FFFFFFu;
    g_tb_last = raw;
    return g_tb_acc;
}

/* ------------------------------ debug logging ----------------------------- */
#if (USE_DEBUG_UART != 0u)
    /* 8N1 LSB-first bit-banged TX, bit edges scheduled as absolute offsets
     * from t0 so per-bit software overhead cannot accumulate (zeta-style)   */
    static void su_tx_byte(uint32 pinPC, uint8 c, uint32 bitcy)
    {
        uint16 frame = (uint16)(((uint16)c << 1) | 0x0600u); /* start,8,2stop */
        uint32 t0;
        uint8  i;
        uint8  ints;

        ints = CyEnterCriticalSection();
        t0 = SU_NOW();
        for (i = 0u; i < 11u; i++)
        {
            if ((frame & 0x01u) != 0u) { CyPins_SetPin(pinPC); }
            else                       { CyPins_ClearPin(pinPC); }
            frame >>= 1;
            while (SU_ELAPSED(t0) < (((uint32)i + 1u) * bitcy)) { }
        }
        CyExitCriticalSection(ints);
    }
    static void dbg_puts(const char *s)
    {
        while (*s != '\0') { su_tx_byte(PIN_DBG_TX, (uint8)*s, DBG_BIT_CY); s++; }
    }
    static char g_dbg[96];
    #define DBG(s)          dbg_puts(s)
    #define DBGF(...)       do { snprintf(g_dbg, sizeof(g_dbg), __VA_ARGS__); \
                                 dbg_puts(g_dbg); } while (0)
#else
    #define DBG(s)          do {} while (0)
    #define DBGF(...)       do {} while (0)
#endif

/* ------------------------------ pin helpers ------------------------------- */
/* whole-byte PC write: drive mode + initial level, BYPASS cleared so the pin
 * follows software even if schematic components claim it                    */
static void pin_out_init(uint32 pinPC, uint8 initialLevel)
{
    uint8 value = CY_PINS_DM_STRONG;
    if (initialLevel != 0u) { value |= CY_PINS_PC_DATAOUT; }
    CY_SET_REG8(pinPC, value);
}

static void pin_write(uint32 pinPC, uint8 level)
{
    if (level != 0u) { CyPins_SetPin(pinPC); }
    else             { CyPins_ClearPin(pinPC); }
}

/* microstep table: USTEP_DIV -> M2 M1 M0 (DRV8825 datasheet table 1) */
#if   (USTEP_DIV == 1u)
    #define M2_LVL 0u
    #define M1_LVL 0u
    #define M0_LVL 0u
#elif (USTEP_DIV == 2u)
    #define M2_LVL 0u
    #define M1_LVL 0u
    #define M0_LVL 1u
#elif (USTEP_DIV == 4u)
    #define M2_LVL 0u
    #define M1_LVL 1u
    #define M0_LVL 0u
#elif (USTEP_DIV == 8u)
    #define M2_LVL 0u
    #define M1_LVL 1u
    #define M0_LVL 1u
#elif (USTEP_DIV == 16u)
    #define M2_LVL 1u
    #define M1_LVL 0u
    #define M0_LVL 0u
#elif (USTEP_DIV == 32u)
    #define M2_LVL 1u
    #define M1_LVL 0u
    #define M0_LVL 1u
#else
    #error "USTEP_DIV must be 1, 2, 4, 8, 16 or 32"
#endif

static void drv_pins_init(const drv_t *d)
{
    pin_out_init(d->en,  1u);   /* outputs disabled while configuring */
    pin_out_init(d->step, 0u);
    pin_out_init(d->dir, 0u);
    pin_out_init(d->slp, 1u);   /* awake */
    pin_out_init(d->rst, 1u);   /* run   */
    pin_out_init(d->m2, M2_LVL);
    pin_out_init(d->m1, M1_LVL);
    pin_out_init(d->m0, M0_LVL);
}

static void drv_enable(const drv_t *d, uint8 on)
{
    pin_write(d->en, (on != 0u) ? 0u : 1u);       /* nENABLE is active low */
}

/* ------------------------------ common init ------------------------------- */
static void hw_init(void)
{
    CyGlobalIntEnable;

    drv_pins_init(&DRV1);
    drv_pins_init(&DRV2);

    /* FAULT1 is open-drain with no pull-up drawn on the carrier: supply one
     * from inside the PSoC, or the pin floats and reads garbage.           */
    CY_SET_REG8(PIN_FAULT, CY_PINS_DM_RES_UP | CY_PINS_PC_DATAOUT);
#if (USE_DEBUG_UART != 0u)
    pin_out_init(PIN_DBG_TX, 1u);                 /* UART idles high */
#endif

    su_timer_start();
    CyDelay(50u);
}

/* ========================================================================= */
#if (WIRING_PROBE_MODE != 0u)
/* ---- WIRING PROBE: verify the PIN MAP with a multimeter -------------------
 * Both drivers stay disabled (EN = 1). Each mapped signal in turn toggles 4x
 * at 1 Hz while its name goes to the debug log; every other pin holds its
 * idle level. Probe the labelled stepper-header pins against GND: the named
 * pin - and only that pin - must swing 0 <-> 3.3 V in time with the log.
 * P2[1] (driver 1 STEP) also flashes the blue LED, a free sanity check.    */
int main(void)
{
    typedef struct { uint32 pc; const char *nm; uint8 idle; } probe_t;
    static const probe_t tab[] = {
        { CYREG_PRT2_PC7, "D1 EN  (P2.7)", 1u },
        { CYREG_PRT2_PC6, "D1 M0  (P2.6)", M0_LVL },
        { CYREG_PRT2_PC5, "D1 M1  (P2.5)", M1_LVL },
        { CYREG_PRT2_PC4, "D1 M2  (P2.4)", M2_LVL },
        { CYREG_PRT2_PC3, "D1 RST (P2.3)", 1u },
        { CYREG_PRT2_PC2, "D1 SLP (P2.2)", 1u },
        { CYREG_PRT2_PC1, "D1 STEP(P2.1)", 0u },
        { CYREG_PRT2_PC0, "D1 DIR (P2.0)", 0u },
        { CYREG_PRT1_PC7,  "D2 EN  (P1.7)",  1u },
        { CYREG_PRT1_PC6,  "D2 M0  (P1.6)",  M0_LVL },
        { CYREG_PRT1_PC5,  "D2 M1  (P1.5)",  M1_LVL },
        { CYREG_PRT1_PC4,  "D2 M2  (P1.4)",  M2_LVL },
        { CYREG_PRT1_PC2,  "D2 RST (P1.2)",  1u },
        { CYREG_PRT12_PC2, "D2 SLP (P12.2)", 1u },
        { CYREG_PRT12_PC3, "D2 STEP(P12.3)", 0u },
        { CYREG_PRT12_PC4, "D2 DIR (P12.4)", 0u },
    };
    uint8 i, k;

    hw_init();
    DBG("\r\n=== WIRING PROBE: drivers disabled, 1 Hz walk ===\r\n");
    DBGF("FAULT1 (P12.5) reads %u (0 = a driver is faulted)\r\n",
         (unsigned)(FAULT_ACTIVE() ? 0u : 1u));

    /* silence unused-function warnings for the spin-only helpers */
    (void)tb_reset; (void)tb_now32; (void)drv_enable;

    for (;;)
    {
        for (i = 0u; i < (sizeof(tab) / sizeof(tab[0])); i++)
        {
            DBGF("%s toggling...\r\n", tab[i].nm);
            for (k = 0u; k < 4u; k++)
            {
                pin_write(tab[i].pc, (uint8)(tab[i].idle == 0u));
                CyDelay(500u);
                pin_write(tab[i].pc, tab[i].idle);
                CyDelay(500u);
            }
        }
        DBG("--- pass done, restarting ---\r\n");
    }
}

#else
/* ------------------------------ spin engine ------------------------------- */
/* Free-running rotation with a startup ramp, then constant cruise. Ramp is
 * the AVR446 / David Austin integer recurrence:
 *   c(n) = c(n-1) - 2*c(n-1)/(4n+1),  c(0) = F*sqrt(2/a)
 * where c is the step period in BUS_CLK cycles. No deceleration: the motor
 * is never asked to stop.                                                   */
typedef struct
{
    const drv_t *d;
    uint32 next_due;    /* tb_now32() time of the next step  */
    int32  c;           /* current step period, cycles       */
    uint32 n;           /* steps spent accelerating          */
    uint32 steps;       /* lifetime step count               */
} spin_t;

static uint32 g_c0;     /* first step period, computed once in main */

static uint32 isqrt64(uint64 v)
{
    uint64 r = 0u;
    uint64 bit = (uint64)1u << 62;
    while (bit > v) { bit >>= 2; }
    while (bit != 0u)
    {
        if (v >= (r + bit)) { v -= r + bit; r = (r >> 1) + bit; }
        else                { r >>= 1; }
        bit >>= 2;
    }
    return (uint32)r;
}

/* one brief low notch; the rising edge at its end is the step. STEP idles
 * HIGH, so the LED (driver 1) stays lit while steps are being commanded    */
static void step_pulse(const drv_t *d)
{
    CyPins_ClearPin(d->step);
    CyDelayUs(STEP_LOW_US);
    CyPins_SetPin(d->step);
}

static void spin_init(spin_t *s, const drv_t *d, uint8 dir, uint32 t)
{
    s->d        = d;
    s->c        = (int32)g_c0;
    s->n        = 0u;
    s->steps    = 0u;
    s->next_due = t;
    pin_write(d->dir, dir);
}

static void spin_service(spin_t *s, uint32 t)
{
    int32 late = (int32)(t - s->next_due);

    if (late < 0) { return; }

    step_pulse(s->d);
    s->steps++;

    if (s->c > (int32)C_MIN)
    {   /* still ramping */
        s->n++;
        s->c -= (2 * s->c) / ((int32)(4u * s->n) + 1);
        if (s->c < (int32)C_MIN) { s->c = (int32)C_MIN; }
    }

    /* A blocking debug line can push us past a deadline. Resync instead of
     * firing a catch-up burst, which would jerk the rotor out of sync.     */
    if (late > s->c) { s->next_due = t + (uint32)s->c; }
    else             { s->next_due += (uint32)s->c; }
}

/* ---- normal build: both motors turn forever ------------------------------ */
int main(void)
{
    spin_t a, b;
    uint32 t, next_log;
    uint8  fault, last_fault = 0xFFu;

    hw_init();
    g_c0 = isqrt64(((uint64)BCLK__BUS_CLK__HZ * (uint64)BCLK__BUS_CLK__HZ * 2u)
                   / ACCEL_SPS2);

    DBG("\r\n=== quad-carrier dual DRV8825: continuous spin ===\r\n");
    DBGF("ustep=1/%u, cruise=%lu steps/s, ramp=%lu steps/s^2\r\n",
         (unsigned)USTEP_DIV, (unsigned long)STEP_SPS, (unsigned long)ACCEL_SPS2);

    /* Read FAULT before enabling anything: with VMOT missing the DRV8825
     * sits in undervoltage lockout and holds this low, which is the fastest
     * way to tell "no motor supply" apart from "motor will not turn".      */
    DBGF("FAULT1 at boot: %s\r\n",
         FAULT_ACTIVE() ? "LOW - driver faulted (VMOT missing? overcurrent?)"
                        : "high - drivers OK");

    drv_enable(&DRV1, 1u);
    drv_enable(&DRV2, 1u);
    CyDelay(2u);

    tb_reset();
    t = tb_now32();
    spin_init(&a, &DRV1, DIR_1, t);
    spin_init(&b, &DRV2, DIR_2, t);
    CyDelayUs(DIR_SETUP_US);
    next_log = t + LOG_CY;
    DBG("spinning\r\n");

    for (;;)
    {
        t = tb_now32();
        spin_service(&a, t);
        spin_service(&b, t);

        if ((int32)(t - next_log) >= 0)
        {
            fault = FAULT_ACTIVE() ? 1u : 0u;
            DBGF("d1=%lurev d2=%lurev fault=%u\r\n",
                 (unsigned long)(a.steps / STEPS_PER_REV),
                 (unsigned long)(b.steps / STEPS_PER_REV),
                 (unsigned)fault);
            if (fault != last_fault)
            {
                DBG(fault ? "  !! FAULT asserted (shared line: either driver)\r\n"
                          : "  fault cleared\r\n");
                last_fault = fault;
            }
            next_log += LOG_CY;
        }
    }
}
#endif

/* [] END OF FILE */
