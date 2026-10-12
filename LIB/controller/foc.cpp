#include "foc.hpp"
#include "arm_math.h"
#include <cmath>

FOC::FOC() {
}

void FOC::set_measure(
    const float& c1, const float& c2, const float& c3, const float& vbus, const float& rad, const bool& rad_is_trustable) {
        if(rad_is_trustable)
            this->rad=rad;
        this->c1=c1;
        this->c2=c2;
        this->c3=c3;
        this->vbus=vbus;
}

bool FOC::get_output(float& v1, float& v2, float& v3) {
    float ix,iy;
    ix=2.0f/3.0f*(c1*1.0f-c2*0.5f-c3*0.5f);         //将三相电流变为正交坐标系下的电流
    iy=2.0f/3.0f*(c1*0.0f+c2*0.866025404f-c3*0.866025404f);

    float erad=(rad*n-offset);
    float s,c;
    arm_sin_cos_f32(erad,&s,&c);   //将三相电流变为正交坐标系下的电流
    float id=c*ix-s*iy;
    float iq=s*ix+c*iy;

    float vq=iq_pid.update(iq,exp_iq_);
    float vd=id_pid.update(id,0.0f);
    
    s=-s;   //得到逆矩阵，用于将电压转回静止坐标系
    float vx=c*vd-s*vq;
    float vy=s*vd+c*vq;

    v1=vx;
    v2=-0.5f*vx+0.866025404f*vy;
    v3=-0.5f*vx-0.866025404f*vy;
    
    return true;
}

void FOC::set_controller_reference(void* ref) {
    exp_iq_=*static_cast<float*>(ref);              //在FOC模式下，将ref视作q轴电流参考值
}
