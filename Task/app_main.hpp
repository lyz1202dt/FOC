#ifndef __APP_MAIN_H__
#define __APP_MAIN_H__


#include "../LIB/motor/motor.hpp"

extern Motor motor;

#ifdef __cplusplus
extern "C" {
#endif



void app_main(void* param);

#ifdef __cplusplus
}
#endif

#endif
