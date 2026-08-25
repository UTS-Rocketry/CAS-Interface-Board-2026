#ifndef AIRBRAKE_TEST_H
#define AIRBRAKE_TEST_H

#include <stdint.h>

#ifdef AIRBRAKE_TEST_SEQUENCE

void airbrake_test_reset(void);
void airbrake_test_update(uint32_t now);

#endif /* AIRBRAKE_TEST_SEQUENCE */

#endif /* AIRBRAKE_TEST_H */