#include "foc.hpp"
#include "arm_math.h"
#include "hardware.hpp"
#include <cmath>

FOC::FOC(Flash &flash) : Controller(flash) {
    load_config();
    flash.read_value("foc/pid/kp",iq_pid.kp_);
    flash.read_value("foc/pid/ki",iq_pid.ki_);
    flash.read_value("foc/pid/kp",id_pid.kp_);
    flash.read_value("foc/pid/ki",id_pid.ki_);
    iq_pid.output_limit_=3.0f;  //安全起见
    id_pid.output_limit_=3.0f;
    iq_pid.inter_limit_=50.0f;
    id_pid.inter_limit_=50.0f;
    iq_pid.reset();
    id_pid.reset();
}

void FOC::load_config() {
    int32_t pole_pairs      = 0;
    float mechanical_offset = 0.0f;
    flash.read_value("foc/n", pole_pairs);
    flash.read_value("foc/offset", mechanical_offset);
    const uint32_t interrupt_mask = __get_PRIMASK();
    __disable_irq();
    n              = pole_pairs;
    offset         = mechanical_offset;
    config_loaded_ = true;
    __set_PRIMASK(interrupt_mask);
}

void FOC::reset() {
    iq_pid.reset();
    id_pid.reset();
    cur_iq_ = 0.0f;
    c1 = c2 = c3 = vbus = rad =  0.0f;
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
    v1 = v2 = v3 = 0.0f;
    if (!config_loaded_) {
        return false;
    }
    float ix,iy;
    ix=2.0f/3.0f*(c1*1.0f-c2*0.5f-c3*0.5f);         //将三相电流变为正交坐标系下的电流
    iy=2.0f/3.0f*(c1*0.0f+c2*0.866025404f-c3*0.866025404f);

    // 校准 offset 存储的是机械角，与 VOC 使用相同的电角度换算。
    constexpr float kTwoPi = 6.28318530717958647692f;
    const float erad = std::fmod((rad - offset) * static_cast<float>(n), kTwoPi);
    if (!std::isfinite(erad)) {
        return false;
    }
    float s,c;
    s=arm_sin_f32(-erad);
    c=arm_cos_f32(-erad);
    float id=c*ix-s*iy;
    float iq=s*ix+c*iy;

    //记录
    cur_iq_=iq;

    float vq=iq_pid.update(iq,exp_iq_);
    float vd=id_pid.update(id,0.0f);

    //电压限幅，但是保证电压矢量方向不变
    float voltage_mag;
    arm_sqrt_f32(vq*vq+vd*vd, &voltage_mag);
    const float safe_voltage=0.9f*0.866025404f*vbus;
    if(voltage_mag>safe_voltage)
    {
        float k=safe_voltage/voltage_mag;
        vq=vq*k;
        vd=vd*k;
    }
    
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
