#include "controller.hpp"
//IDEL 空闲模式，永远不输出有效电压

class IDEL : public Controller{
public:
    IDEL(Flash &flash);
    ~IDEL()=default;
    void set_measure(
        const float& c1, const float& c2, const float& c3, const float& vbus, const float& rad, const bool& rad_is_trustable) override;
    bool get_output(float& v1, float& v2, float& v3) override;
    void set_controller_reference(void* ref) override;
};