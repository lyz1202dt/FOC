#include "voc.hpp"
#include "controller.hpp"

VOC::VOC(Flash &flash): Controller(flash)
{

}

void VOC::set_measure(const float& c1, const float& c2, const float& c3, const float& vbus, const float& rad, const bool& rad_is_trustable){

}

bool VOC::get_output(float& v1, float& v2, float& v3){

}

void VOC::set_controller_reference(void* ref){

}
