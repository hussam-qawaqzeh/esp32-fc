#pragma once

#include <stdint.h>

namespace Espfc {
namespace Control {

static constexpr float FAILSAFE_PWM_MIN = 1000.0f;
static constexpr float FAILSAFE_PWM_MID = 1500.0f;
static constexpr float FAILSAFE_PWM_MAX = 2000.0f;

struct FailsafeCommand
{
  float roll = FAILSAFE_PWM_MID;
  float pitch = FAILSAFE_PWM_MID;
  float yaw = FAILSAFE_PWM_MID;
  float thrust = FAILSAFE_PWM_MIN;
};

}
}
