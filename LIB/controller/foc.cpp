#include "foc.hpp"

FOC::FOC() {
}

void FOC::set_measure(const float& c1, const float& c2, const float& c3, const float& vbus, const float& rad, const bool &rad_is_trustable) {
}
bool FOC::get_output(float& v1, float& v2, float& v3) {
    v1 = 0.0f;
    v2 = 0.0f;
    v3 = 0.0f;
    return true;
}

void FOC::set_reference_current(float iq,float id)
{
    exp_id_=id;
    exp_iq_=iq;
}
