#include "servo.h"
#include "main.h"

/*
 * Routing (Kestrel):
 *   Airbrakes : PB1 = TIM3_CH4, AF2
 *   Roll/CAS  : PB0 = TIM3_CH3, AF2   (compiled only if SERVO_ENABLE_ROLL)
 *
 * TIM3 is on APB1. On this board SYSCLK=72 MHz, APB1=/2 -> PCLK1=36 MHz,
 * and the APB1 timer doubler gives TIM3 a 72 MHz clock.
 *   PSC = 72-1 -> 1 MHz tick (1 us/count)
 *   ARR = 20000-1 -> 20 ms period = 50 Hz
 *   CCRx in microseconds == pulse width.
 * The actual clock is read at runtime (see apb1_timer_clk) so the 1 MHz tick
 * holds even if the clock tree changes.
 */

#define TICK_HZ         1000000u            /* 1 MHz -> 1 us/tick */
#define PERIOD_US       20000u              /* 20 ms -> 50 Hz */

/*
 * Compute the TIM3 (APB1) timer clock at runtime instead of hardcoding it.
 * Per the F4 clock tree: if the APB1 prescaler is 1, the timer clock equals
 * PCLK1; otherwise the timers see 2 x PCLK1. This avoids a silent timing bug
 * if the project's clock config differs from the assumed value.
 */
static uint32_t apb1_timer_clk(void)
{
    RCC_ClkInitTypeDef clk;
    uint32_t flash_latency;
    HAL_RCC_GetClockConfig(&clk, &flash_latency);

    uint32_t pclk1 = HAL_RCC_GetPCLK1Freq();
    /* APB1 prescaler == 1 -> timers run at PCLK1; else at 2x PCLK1 */
    if (clk.APB1CLKDivider == RCC_HCLK_DIV1) {
        return pclk1;
    }
    return pclk1 * 2u;
}

static inline uint16_t clamp_us(uint16_t us)
{
    if (us < SERVO_US_MIN) return SERVO_US_MIN;
    if (us > SERVO_US_MAX) return SERVO_US_MAX;
    return us;
}

void servo_init(void)
{
    /* --- Clock --- *
     * NOTE: GPIO pins PB0 (roll) and PB1 (airbrakes) are already configured
     * as TIM3 AF2 by CubeMX's MX_GPIO_Init(), which runs before this. We only
     * own the timer peripheral here. Do not re-init the GPIO.
     */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    /* --- Timebase: 1 MHz tick, 20 ms period --- */
    TIM3->PSC = (apb1_timer_clk() / TICK_HZ) - 1u;   /* -> 1 MHz regardless of clock cfg */
    TIM3->ARR = PERIOD_US - 1u;                       /* 19999 */

    /* --- CH4 (airbrakes): PWM mode 1, preload, enable output --- */
    TIM3->CCMR2 &= ~TIM_CCMR2_OC4M;
    TIM3->CCMR2 |=  (0x6u << TIM_CCMR2_OC4M_Pos);   /* PWM mode 1 */
    TIM3->CCMR2 |=  TIM_CCMR2_OC4PE;                /* preload CCR4 */
    TIM3->CCER  |=  TIM_CCER_CC4E;                  /* enable CH4 output */
    TIM3->CCR4   =  SERVO_US_MID;

#if SERVO_ENABLE_ROLL
    /* --- CH3 (roll): PWM mode 1, preload, enable output --- */
    TIM3->CCMR2 &= ~TIM_CCMR2_OC3M;
    TIM3->CCMR2 |=  (0x6u << TIM_CCMR2_OC3M_Pos);   /* PWM mode 1 */
    TIM3->CCMR2 |=  TIM_CCMR2_OC3PE;                /* preload CCR3 */
    TIM3->CCER  |=  TIM_CCER_CC3E;                  /* enable CH3 output */
    TIM3->CCR3   =  SERVO_US_MID;
#endif

    /* --- Latch preloaded values, enable ARR preload, start counter --- */
    TIM3->EGR  |= TIM_EGR_UG;       /* force update: load PSC/ARR/CCR now */
    TIM3->CR1  |= TIM_CR1_ARPE;     /* auto-reload preload */
    TIM3->CR1  |= TIM_CR1_CEN;      /* go */

    #ifdef DEBUG
        printf("servo_init done\r\n");
        printf("  APB1 timer clk = %lu Hz\r\n", apb1_timer_clk());
        printf("  PSC=%lu ARR=%lu\r\n", (unsigned long)TIM3->PSC, (unsigned long)TIM3->ARR);
        printf("  CCMR2=0x%08lX CCER=0x%08lX\r\n", (unsigned long)TIM3->CCMR2, (unsigned long)TIM3->CCER);
        printf("  CR1=0x%08lX CCR4=%lu\r\n", (unsigned long)TIM3->CR1, (unsigned long)TIM3->CCR4);
    #endif
}

void servo_set_us(servo_id_t id, uint16_t us)
{
    us = clamp_us(us);
    switch (id) {
        case SERVO_AIRBRAKE: TIM3->CCR4 = us; break;
#if SERVO_ENABLE_ROLL
        case SERVO_ROLL:     TIM3->CCR3 = us; break;
#endif
        default: break;
    }
}

void servo_set_deg(servo_id_t id, uint8_t deg)
{
    if (deg > 180u) deg = 180u;
    uint16_t us = (uint16_t)(SERVO_US_MIN +
                  ((uint32_t)deg * (SERVO_US_MAX - SERVO_US_MIN)) / 180u);
    servo_set_us(id, us);
}

void servo_set_fraction(servo_id_t id, float frac)
{
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;
    uint16_t us = (uint16_t)(SERVO_US_MIN +
                  frac * (float)(SERVO_US_MAX - SERVO_US_MIN));
    servo_set_us(id, us);
}