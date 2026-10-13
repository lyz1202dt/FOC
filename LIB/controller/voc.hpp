#include "controller.hpp"

//VOC————Voltage Open Circle 电压开环测试

class VOC : public Controller{
public:
    VOC(Flash &flash);
    ~VOC()=default;
    void set_measure(
        const float& c1, const float& c2, const float& c3, const float& vbus, const float& rad, const bool& rad_is_trustable) override;
    bool get_output(float& v1, float& v2, float& v3) override;
    void set_controller_reference(void* ref) override;
};
