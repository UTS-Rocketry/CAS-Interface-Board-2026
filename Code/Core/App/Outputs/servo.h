#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

/*
 * Servo PWM driver for STM32F405 (Kestrel airbrake / CAS board)
 *
 * One shared timer (TIM3) generates the 50 Hz / 20 ms period. Two channels
 * ride that period with independent pulse widths:
 *
 *   Airbrakes servo : PB1 -> TIM3_CH4 (AF2)   [active now]
 *   Roll servo (CAS): PB0 -> TIM3_CH3 (AF2)   [scaffolded, gated off]
 *
 * Timer ticks at 1 MHz, so the CCR value == pulse width in microseconds.
 *
 * Roll control is not populated/used on this board spin. Compile it in by
 * setting SERVO_ENABLE_ROLL to 1 once the CAS actuator exists.
 *
 * SAFETY: setters clamp to [SERVO_US_MIN, SERVO_US_MAX]. Find each servo's
 * true mechanical limits on the bench BEFORE widening these. Driving past the
 * mechanical stop stalls the motor and pulls stall current.
 */

#define SERVO_ENABLE_ROLL   0      /* flip to 1 when CAS roll actuator is fitted */

#define SERVO_US_MIN   1000u       /* fully stowed  / one extreme */
#define SERVO_US_MAX   2000u       /* fully deployed / other extreme */
#define SERVO_US_MID   1500u       /* center */

/* Logical channel identifiers (decoupled from the hardware TIM channel). */
typedef enum {
    SERVO_AIRBRAKE = 0,
#if SERVO_ENABLE_ROLL
    SERVO_ROLL,
#endif
    SERVO_COUNT
} servo_id_t;

/* Initialize TIM3 + GPIO and start the 50 Hz output on enabled channels.
 * Leaves every channel at SERVO_US_MID. */
void servo_init(void);

/* Set pulse width directly in microseconds. Clamped to [MIN, MAX]. */
void servo_set_us(servo_id_t id, uint16_t us);

/* Set angle 0..180 deg. Convenience wrapper over servo_set_us. */
void servo_set_deg(servo_id_t id, uint8_t deg);

/* Set deploy fraction 0.0 (stowed) .. 1.0 (fully deployed).
 * This is the interface the airbrake controller will call.
 * Maps linearly across [SERVO_US_MIN, SERVO_US_MAX]. */
void servo_set_fraction(servo_id_t id, float frac);

#endif /* SERVO_H */