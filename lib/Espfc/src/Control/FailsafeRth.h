#pragma once

#include "Control/FailsafeTypes.h"
#include "Model.h"

namespace Espfc {
namespace Control {

enum class FailsafeRthAction
{
  Continue,
  Land,
  Disarm
};

class FailsafeRth
{
  public:
    explicit FailsafeRth(Model& model);

    bool canUse() const;
    void reset();
    FailsafeRthAction update(FailsafeCommand& command);

  private:
    float gpsCourseDegrees() const;
    void buildCommand(FailsafeCommand& command);

    Model& _model;
    bool _active;
    bool _pitchFlipped;
    float _bearing;
    float _pitchSign;

    static constexpr float MIN_SPEED = 0.5f;
    static constexpr float PITCH_GAIN = 4.0f;
    static constexpr float YAW_GAIN = 1.0f / 60.0f;
    static constexpr float MAX_CRAB_ANGLE = 20.0f;
};

}
}
