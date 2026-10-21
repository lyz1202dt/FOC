#include "foc.hpp"
#include "arm_math.h"
#include <cmath>

FOC::FOC(Flash &flash) : Controller(flash) {
    n=1;        //默认值
    offset=0;
    flash.read_value("foc/n",n);
    flash.read_value("foc/offset",offset);
}

void FOC::reset() {
    iq_pid.reset();
    id_pid.reset();
    cur_iq_ = 0.0f;
    c1 = c2 = c3 = vbus = rad = 0.0f;
    first_run = true;
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
    s=arm_sin_f32(-erad);
    c=arm_cos_f32(-erad);
    float id=c*ix-s*iy;
    float iq=s*ix+c*iy;

    //记录
    cur_iq_=iq;

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

void FOC::controller_reference(void* ref,size_t size) {
    exp_iq_=*static_cast<float*>(ref);              //在FOC模式下，将ref视作q轴电流参考值
}

void FOC::controller_status(void* ref,size_t size){
    if(size==sizeof(float))
        *static_cast<float*>(ref)=cur_iq_;
}