#pragma once

#include "encoder.hpp"

class VirtualEncoder : public Encoder {
public:
    explicit VirtualEncoder(Flash& flash);
    bool init() override;
    bool get_value(float& value, uint32_t max_age_us, uint64_t* sample_time_us = nullptr) override;
};
