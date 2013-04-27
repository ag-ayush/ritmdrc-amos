#ifndef _POWER_H_
#define _POWER_H_

#include <WProgram.h>


#ifdef __cplusplus
extern "C" {
#endif

int power_get_voltage_left();
int power_get_current_left();
int power_get_voltage_right();
int power_get_current_right();

#ifdef __cplusplus
}
#endif

#endif // _POWER_H_

