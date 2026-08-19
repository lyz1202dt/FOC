#include "controller.hpp"

class FOC : public Controller {
    FOC();
    void set_measure(const float& c1, const float& c2, const float& c3, const float& rad, const float& vbus) override;
    bool get_output(float& v1, float& v2, float &v3) override;
};
