#include "Control/FailsafeLand.h"
#include "Utils/Math.hpp"
#include <Arduino.h>
#include <cmath>

namespace Espfc {
namespace Control {

FailsafeLand::FailsafeLand(Model& model): _model(model) {}

bool FailsafeLand::canUse() const
{
  return _model.baroActive();
}

void FailsafeLand::reset()
{
}

bool FailsafeLand::update(FailsafeCommand& command)
{
  if(_model.state.failsafe.phase != FC_FAILSAFE_LANDING)
  {
    _model.state.failsafe.phase = FC_FAILSAFE_LANDING;
    _model.state.failsafe.timeout = millis();
  }

  command.roll = FAILSAFE_PWM_MID;
  command.pitch = FAILSAFE_PWM_MID;
  command.yaw = FAILSAFE_PWM_MID;
  command.thrust = Utils::clamp((int)_model.config.failsafe.throttle, (int)FAILSAFE_PWM_MIN, (int)FAILSAFE_PWM_MAX);

  const uint32_t elapsed = millis() - _model.state.failsafe.timeout;
  const uint32_t maxLandingTimeMs = (uint32_t)_model.config.failsafe.offDelay * 100UL;
  const bool nearGround = _model.state.altitude.height < LANDING_HEIGHT && std::fabs(_model.state.altitude.vario) < LANDING_VARIO;

  return (nearGround && elapsed >= LANDING_MIN_TIME_MS) || (maxLandingTimeMs > 0 && elapsed >= maxLandingTimeMs);
}

}
}
