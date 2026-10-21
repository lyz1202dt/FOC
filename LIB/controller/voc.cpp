#include "voc.hpp"
#include "controller.hpp"
#include "arm_math.h"

VOC::VOC(Flash &flash): Controller(flash)
{

}

void VOC::reset() {
    vir_ecc_rad = 0.0f;
}

void VOC::set_measure(const float& c1, const float& c2, const float& c3, const float& vbus, const float& rad, const bool& rad_is_trustable){

}

bool VOC::get_output(float& v1, float& v2, float& v3){
    float vq=1.0f;  //测试电压
    float vd=0.0f;

    vir_ecc_rad+=0.0001f;
    if(vir_ecc_rad>=3.14159265f*2)
        vir_ecc_rad=0.0f;
    
    float s,c;
    s=arm_sin_f32(-vir_ecc_rad);
    c=arm_cos_f32(-vir_ecc_rad);

    float vx=c*vd-s*vq;
    float vy=s*vd+c*vq;

    v1=vx;
    v2=-0.5f*vx+0.866025404f*vy;
    v3=-0.5f*vx-0.866025404f*vy;
    
    return true;
}

void VOC::controller_reference(void* ref,size_t size){

}
