#pragma once
#include "motion_strategy.hpp"

class ManualControl : public IMotionStrategy {
public:
    ManualControl() = default;
    ~ManualControl() override = default;

    MotionOutput compute(const StateSnapshot& state, float dt_s, uint32_t loop_counter) override;
};
