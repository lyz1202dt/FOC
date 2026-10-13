#include "idel.hpp"

IDEL::IDEL(Flash &flash): Controller(flash)
{

}

void IDEL::set_measure(const float& c1, const float& c2, const float& c3, const float& vbus, const float& rad, const bool& rad_is_trustable){

}

bool IDEL::get_output(float& v1, float& v2, float& v3){     //输出0电压
    v1=0.0f;
    v2=0.0f;
    v3=0.0f;
}

void IDEL::set_controller_reference(void* ref){

}
