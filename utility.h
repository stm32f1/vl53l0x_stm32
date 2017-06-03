#ifndef UTILITY_H
#define UTILITY_H

#include <math.h>
#include <stdint.h>
#include "device.h"

void delay_us(unsigned int us);
void delay_ms(unsigned int msec);
void delay_s(unsigned int sec);
void pipipi(int count);
void pii(int count);

static inline unsigned int sum_u16(__I unsigned int *array_name, size_t num) __attribute__((always_inline));
static inline unsigned int sum_u16(__I unsigned int *array_name, size_t num)
{
	unsigned int total = 0;
	for (size_t i = 0; i < num; i++) {
		total += array_name[i];
	}
	return total;
}


static inline double todegrees(double radians) __attribute__((always_inline));
static inline double todegrees(double radians)
{
	return 180 * radians / M_PI;
}


static inline double toradians(double degrees) __attribute__((always_inline));
static inline double toradians(double degrees)
{
	return M_PI * degrees / 180;
}

void inform_error(void);

#endif /* UTILITY_H */
