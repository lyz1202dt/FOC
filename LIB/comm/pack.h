#ifndef __PACK_H__
#define __PACK_H__


extern "C"{

#include <stdint.h>


#pragma pack(1)


typedef struct{
    uint32_t cmd;
    uint32_t size;
    uint8_t data[];
}Command;




typedef struct{
    float position;
    float velocity;
    float torque;
    float kp;
    float kd;
}MotorCommand;

typedef struct{
    float position;
    int32_t r;
    float velocity;
    float torque;
    uint32_t status;
}MotorStatus;



#pragma pack()

}

#endif