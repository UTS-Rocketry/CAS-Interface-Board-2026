#include "airbrake_test.h"

#ifdef AIRBRAKE_TEST_SEQUENCE

#include "servo.h"
#include "flight_state.h"

/* How long to wait after entering STATE_COAST before starting the test sequence */
#define AIRBRAKE_TEST_COAST_DELAY_MS  1500

/* How long to hold each stage of the sequence */
#define AIRBRAKE_TEST_STAGE_MS        2000

typedef enum {
    AIRBRAKE_TEST_IDLE,
    AIRBRAKE_TEST_HALF,
    AIRBRAKE_TEST_FULL,
    AIRBRAKE_TEST_DONE
} AirbrakeTestState;

static AirbrakeTestState airbrake_test_state = AIRBRAKE_TEST_IDLE;
static uint32_t airbrake_test_t0 = 0;
static uint32_t coast_entry_time = 0;
static uint8_t  was_in_coast = 0;

void airbrake_test_reset(void) {
    airbrake_test_state = AIRBRAKE_TEST_IDLE;
    airbrake_test_t0 = 0;
    coast_entry_time = 0;
    was_in_coast = 0;
}

void airbrake_test_update(uint32_t now) {
    if (FSM_get_state() != STATE_COAST) {
        was_in_coast = 0;
        return;
    }

    /* mark the moment we entered coast, once */
    if (!was_in_coast) {
        coast_entry_time = now;
        was_in_coast = 1;
    }

    switch (airbrake_test_state) {
        case AIRBRAKE_TEST_IDLE:
            if (now - coast_entry_time >= AIRBRAKE_TEST_COAST_DELAY_MS) {
                servo_set_us(SERVO_AIRBRAKE, SERVO_US_MID);
                airbrake_test_t0 = now;
                airbrake_test_state = AIRBRAKE_TEST_HALF;
            }
            break;

        case AIRBRAKE_TEST_HALF:
            if (now - airbrake_test_t0 >= AIRBRAKE_TEST_STAGE_MS) {
                servo_set_us(SERVO_AIRBRAKE, SERVO_US_MAX);
                airbrake_test_t0 = now;
                airbrake_test_state = AIRBRAKE_TEST_FULL;
            }
            break;

        case AIRBRAKE_TEST_FULL:
            if (now - airbrake_test_t0 >= AIRBRAKE_TEST_STAGE_MS) {
                servo_set_us(SERVO_AIRBRAKE, SERVO_US_MIN);
                airbrake_test_state = AIRBRAKE_TEST_DONE;
            }
            break;

        case AIRBRAKE_TEST_DONE:
        default:
            break;
    }
}

#endif /* AIRBRAKE_TEST_SEQUENCE */