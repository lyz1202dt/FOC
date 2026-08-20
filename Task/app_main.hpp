#ifndef __APP_MAIN_H__
#define __APP_MAIN_H__


#ifdef __cplusplus

#include "../LIB/motor/motor.hpp"

extern Motor motor;

extern "C" {
#endif

void app_main(void* param);

#ifdef __cplusplus
}
#endif

#endif
