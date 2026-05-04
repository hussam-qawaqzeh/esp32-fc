#pragma once

#include "Control/FailsafeTypes.h"
#include "Model.h"

namespace Espfc {
namespace Control {

class FailsafeLand
{
  public:
    explicit FailsafeLand(Model& model);

    bool canUse() const;
    void reset();
    bool update(FailsafeCommand& command);

  private:
    Model& _model;

    static constexpr float LANDING_HEIGHT = 0.35f;
    static constexpr float LANDING_VARIO = 0.25f;
    static constexpr uint32_t LANDING_MIN_TIME_MS = 1500;
};

}
}
