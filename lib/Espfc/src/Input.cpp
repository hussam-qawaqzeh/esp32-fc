
#include "Input.h"
#include "Utils/Math.hpp"
#include "Utils/MemoryHelper.h"
#include <cmath>

namespace Espfc {

namespace {

float wrapAngle180(float angle)
{
  while(angle > 180.f) angle -= 360.f;
  while(angle < -180.f) angle += 360.f;
  return angle;
}

float wrapAngle360(float angle)
{
  while(angle >= 360.f) angle -= 360.f;
  while(angle < 0.f) angle += 360.f;
  return angle;
}

float blendAngle(float current, float target, float alpha)
{
  return wrapAngle360(current + wrapAngle180(target - current) * Utils::clamp(alpha, 0.f, 1.f));
}

float normalizedToPwm(float value)
{
  return PWM_RANGE_MID + Utils::clamp(value, -1.f, 1.f) * ((PWM_RANGE_MAX - PWM_RANGE_MIN) * 0.5f);
}

float altHoldRateToInput(float rate)
{
  float input = 0.f;
  if(rate < 0.f)
  {
    input = Utils::map(Utils::clamp(rate, -2.f, 0.f), -2.f, 0.f, -1.f, 0.f);
  }
  else if(rate > 0.f)
  {
    input = Utils::map(Utils::clamp(rate, 0.f, 4.f), 0.f, 4.f, 0.f, 1.f);
  }

  if(input > 0.f) input = Utils::clamp(input + 0.1f, 0.f, 1.f);
  if(input < 0.f) input = Utils::clamp(input - 0.1f, -1.f, 0.f);

  return input;
}

float gpsCourseDegrees(const Model& model)
{
  if(model.state.gps.velocity.raw.heading != 0)
  {
    return wrapAngle360(model.state.gps.velocity.raw.heading * 1e-5f);
  }

  const float north = model.state.gps.velocity.raw.north * 0.001f;
  const float east = model.state.gps.velocity.raw.east * 0.001f;
  if(std::fabs(north) < 0.001f && std::fabs(east) < 0.001f)
  {
    return 0.f;
  }

  return wrapAngle360(Utils::toDeg(std::atan2(east, north)));
}

}

Input::Input(Model& model, TelemetryManager& telemetry):
  _model(model), _telemetry(telemetry), _device(nullptr), _step(0.0f),
  _failsafeRescueActive(false), _failsafeRescuePitchFlipped(false), _failsafeRescueBearing(0.0f), _failsafeRescuePitchSign(-1.0f) {}

int Input::begin()
{
  _device = getInputDevice();
  _model.state.input.channelCount = _device ? _device->getChannelCount() : INPUT_CHANNELS;
  _model.state.input.frameDelta = FRAME_TIME_DEFAULT_US;
  _model.state.input.frameRate = 1000000ul / _model.state.input.frameDelta;
  _model.state.input.frameCount = 0;
  _model.state.input.autoFactor = 1.f / (2.f + _model.config.input.filterAutoFactor * 0.1f);
  switch(_model.config.input.interpolationMode)
  {
    case INPUT_INTERPOLATION_AUTO:
      _model.state.input.interpolationDelta = Utils::clamp(_model.state.input.frameDelta, (uint32_t)4000, (uint32_t)40000) * 0.000001f; // estimate real interval
      break;
    case INPUT_INTERPOLATION_MANUAL:
      _model.state.input.interpolationDelta = _model.config.input.interpolationInterval * 0.001f; // manual interval
      break;
    case INPUT_INTERPOLATION_DEFAULT:
    case INPUT_INTERPOLATION_OFF:
    default:
      _model.state.input.interpolationDelta = FRAME_TIME_DEFAULT_US * 0.000001f;
      break;
  }
  _model.state.input.interpolationStep = _model.state.loopTimer.intervalf / _model.state.input.interpolationDelta;
  _step = 0.0f;
  resetFailsafeRescueState();
  for(size_t c = 0; c < INPUT_CHANNELS; ++c)
  {
    if(_device) _filter[c].begin(FilterConfig(_device->needAverage() ? FILTER_FIR2 : FILTER_NONE, 1), _model.state.loopTimer.rate);
    int16_t v = c == AXIS_THRUST ? PWM_RANGE_MIN : PWM_RANGE_MID;
    _model.state.input.raw[c] = v;
    _model.state.input.buffer[c] = v;
    _model.state.input.bufferPrevious[c] = v;
    setInput((Axis)c, v, true, true);
  }
  return 1;
}

int16_t FAST_CODE_ATTR Input::getFailsafeValue(uint8_t c)
{
  const InputChannelConfig& ich = _model.config.input.channel[c];
  switch(ich.fsMode)
  {
    case FAILSAFE_MODE_AUTO:
      return c == AXIS_THRUST ? PWM_RANGE_MIN : PWM_RANGE_MID;
    case FAILSAFE_MODE_SET:
      return ich.fsValue;
    case FAILSAFE_MODE_INVALID:
    case FAILSAFE_MODE_HOLD:
    default:
      return _model.state.input.buffer[c];
  }
}

void FAST_CODE_ATTR Input::setInput(Axis i, float v, bool newFrame, bool noFilter)
{
  const InputChannelConfig& ich = _model.config.input.channel[i];
  if(i <= AXIS_THRUST)
  {
    const float nv = noFilter ? v : _model.state.input.filter[i].update(v);
    _model.state.input.us[i] = nv;
    _model.state.input.ch[i] = Utils::map(nv, ich.min, ich.max, -1.f, 1.f);
  }
  else if(newFrame)
  {
    _model.state.input.us[i] = v;
    _model.state.input.ch[i] = Utils::map(v, ich.min, ich.max, -1.f, 1.f);
  }
}

int FAST_CODE_ATTR Input::update()
{
  if(!_device) return 0;

  uint32_t startTime = micros();

  InputStatus status = readInputs();

  if(!failsafe(status))
  {
    filterInputs(status);
  }

  if(_model.config.debug.mode == DEBUG_PIDLOOP)
  {
    _model.state.debug[1] = micros() - startTime;
  }

  return 1;
}

InputStatus FAST_CODE_ATTR Input::readInputs()
{
  Utils::Stats::Measure readMeasure(_model.state.stats, COUNTER_INPUT_READ);
  uint32_t startTime = micros();

  InputStatus status = _device->update();

  if(_model.config.debug.mode == DEBUG_RX_TIMING)
  {
    _model.state.debug[0] = micros() - startTime;
  }

  if(status == INPUT_IDLE) return status;

  _model.state.input.rxLoss = (status == INPUT_LOST || status == INPUT_FAILSAFE);
  _model.state.input.rxFailSafe = (status == INPUT_FAILSAFE);
  _model.state.input.frameCount++;

  updateFrameRate();

  processInputs();

  if(_model.config.debug.mode == DEBUG_RX_SIGNAL_LOSS)
  {
    _model.state.debug[0] = !_model.state.input.rxLoss;
    _model.state.debug[1] = _model.state.input.rxFailSafe;
    _model.state.debug[2] = _model.state.input.channelsValid;
    _model.state.debug[3] = _model.state.input.lossTime / (100 * 1000);
  }

  return status;
}

void FAST_CODE_ATTR Input::processInputs()
{
  if(_model.state.input.frameCount < 5) return; // ignore few first frames that might be garbage

  uint32_t startTime = micros();

  uint16_t channels[INPUT_CHANNELS];
  _device->get(channels, _model.state.input.channelCount);

  _model.state.input.channelsValid = true;
  for(size_t c = 0; c < _model.state.input.channelCount; c++)
  {
    const InputChannelConfig& ich = _model.config.input.channel[c];

    // remap channels
    int16_t v = _model.state.input.raw[c] = (int16_t)channels[ich.map];

    // adj midrc
    v -= _model.config.input.midRc - PWM_RANGE_MID;

    // adj range
    //float t = Utils::map3((float)v, (float)ich.min, (float)ich.neutral, (float)ich.max, (float)PWM_RANGE_MIN, (float)PWM_RANGE_MID, (float)PWM_RANGE_MAX);
    float t = Utils::mapi(v, ich.min, ich.max, PWM_RANGE_MIN, PWM_RANGE_MAX);

    // filter if required
    t = _filter[c].update(t);
    v = lrintf(t);

    // apply deadband
    if(c < AXIS_THRUST)
    {
      v = Utils::deadband(v - PWM_RANGE_MID, (int)_model.config.input.deadband) + PWM_RANGE_MID;
    }

    // check if inputs are valid, apply failsafe value otherwise
    if(v < _model.config.input.minRc || v > _model.config.input.maxRc)
    {
      v = getFailsafeValue(c);
      if(c <= AXIS_THRUST) _model.state.input.channelsValid = false;
    }

    // update input buffer
    _model.state.input.bufferPrevious[c] = _model.state.input.buffer[c];
    _model.state.input.buffer[c] = v;
  }

  if(_model.config.debug.mode == DEBUG_RX_TIMING)
  {
    _model.state.debug[2] = micros() - startTime;
  }
}

bool FAST_CODE_ATTR Input::failsafe(InputStatus status)
{
  Utils::Stats::Measure readMeasure(_model.state.stats, COUNTER_FAILSAFE);

  if(_model.isSwitchActive(MODE_FAILSAFE))
  {
    failsafeStage2();
    return true;
  }

  if(status == INPUT_RECEIVED)
  {
    failsafeIdle();
    return false;
  }

  if(status == INPUT_FAILSAFE)
  {
    failsafeStage2();
    return true;
  }

  // stage 2 timeout
  _model.state.input.lossTime = micros() - _model.state.input.frameTime;
  if(_model.state.input.lossTime > Utils::clamp((uint32_t)_model.config.failsafe.delay, (uint32_t)2u, (uint32_t)200u) * TENTH_TO_US)
  {
    failsafeStage2();
    return true;
  }

  // stage 1 timeout (100ms)
  if(_model.state.input.lossTime >= 2 * TENTH_TO_US)
  {
    failsafeStage1();
    return true;
  }

  return false;
}

void FAST_CODE_ATTR Input::failsafeIdle()
{
  _model.state.failsafe.phase = FC_FAILSAFE_IDLE;
  _model.state.failsafe.timeout = 0;
  _model.state.input.lossTime = 0;
  resetFailsafeRescueState();
}

void FAST_CODE_ATTR Input::failsafeStage1()
{
  _model.state.failsafe.phase = FC_FAILSAFE_RX_LOSS_DETECTED;
  _model.state.input.rxLoss = true;
  applyFailsafeChannels();
}

void FAST_CODE_ATTR Input::failsafeStage2()
{
  _model.state.input.rxLoss = true;
  _model.state.input.rxFailSafe = true;

  const auto updateLanding = [&](DisarmReason reason)
  {
    const bool justStarted = _model.state.failsafe.phase != FC_FAILSAFE_LANDING;
    if(justStarted)
    {
      _model.state.failsafe.phase = FC_FAILSAFE_LANDING;
      _model.state.failsafe.timeout = millis();
      resetFailsafeRescueState();
    }

    applyFailsafeLandingChannels();

    const uint32_t elapsed = millis() - _model.state.failsafe.timeout;
    const uint32_t maxLandingTimeMs = (uint32_t)_model.config.failsafe.offDelay * 100UL;
    const bool nearGround = _model.state.altitude.height < FAILSAFE_LANDING_HEIGHT && std::abs(_model.state.altitude.vario) < FAILSAFE_LANDING_VARIO;

    if((nearGround && elapsed >= FAILSAFE_LANDING_MIN_TIME_MS) || (maxLandingTimeMs > 0 && elapsed >= maxLandingTimeMs))
    {
      finishFailsafeLanding(reason);
    }
  };

  if(!_model.isModeActive(MODE_ARMED))
  {
    _model.state.failsafe.phase = FC_FAILSAFE_RX_LOSS_DETECTED;
    resetFailsafeRescueState();
    return;
  }

  if(_model.state.failsafe.phase == FC_FAILSAFE_LANDING)
  {
    updateLanding(_model.config.failsafe.procedure == FAILSAFE_PROCEDURE_GPS_RESCUE ? DISARM_REASON_GPS_RESCUE : DISARM_REASON_FAILSAFE);
    return;
  }

  if(_model.config.failsafe.procedure == FAILSAFE_PROCEDURE_GPS_RESCUE && canUseFailsafeRescue())
  {
    const bool justStarted = _model.state.failsafe.phase != FC_FAILSAFE_GPS_RESCUE;
    if(justStarted)
    {
      _model.state.failsafe.phase = FC_FAILSAFE_GPS_RESCUE;
      _model.state.failsafe.timeout = millis();
      resetFailsafeRescueState();
    }

    if(_model.state.gps.distanceToHome <= std::max((int)_model.config.gps.rescueMinDistance, 3))
    {
      if(canUseFailsafeLanding())
      {
        updateLanding(DISARM_REASON_GPS_RESCUE);
      }
      else
      {
        finishFailsafeLanding(DISARM_REASON_GPS_RESCUE);
      }
      return;
    }

    applyFailsafeRescueChannels();

    const uint32_t elapsed = millis() - _model.state.failsafe.timeout;
    const uint32_t maxRescueTimeMs = (uint32_t)_model.config.failsafe.offDelay * 100UL;
    if(maxRescueTimeMs > 0 && elapsed >= maxRescueTimeMs)
    {
      if(canUseFailsafeLanding())
      {
        updateLanding(DISARM_REASON_GPS_RESCUE);
      }
      else
      {
        finishFailsafeLanding(DISARM_REASON_GPS_RESCUE);
      }
    }
    return;
  }

  if(canUseFailsafeLanding())
  {
    updateLanding(DISARM_REASON_FAILSAFE);
    return;
  }

  finishFailsafeLanding();
}

bool Input::canUseFailsafeLanding() const
{
  return _model.baroActive();
}

bool Input::canUseFailsafeRescue() const
{
  if(_model.config.failsafe.procedure != FAILSAFE_PROCEDURE_GPS_RESCUE) return false;
  if(!_model.gpsActive() || !_model.state.gps.isHomeValid()) return false;
  if(_model.state.gps.numSats < _model.config.gps.rescueMinSats) return false;
  if(!_model.accelActive()) return false;
  if(_model.config.gps.rescueSanityChecks && _model.state.gps.distanceToHome < _model.config.gps.rescueMinDistance) return false;
  return true;
}

void Input::applyFailsafeChannels()
{
  for(size_t i = 0; i < _model.state.input.channelCount; i++)
  {
    setInput((Axis)i, getFailsafeValue(i), true, true);
  }
}

void Input::applyFailsafeLandingChannels()
{
  setInput(AXIS_ROLL, PWM_RANGE_MID, true, true);
  setInput(AXIS_PITCH, PWM_RANGE_MID, true, true);
  setInput(AXIS_YAW, PWM_RANGE_MID, true, true);
  setInput(AXIS_THRUST, Utils::clamp((int)_model.config.failsafe.throttle, (int)PWM_RANGE_MIN, (int)PWM_RANGE_MAX), true, true);
}

void Input::applyFailsafeRescueChannels()
{
  const float homeBearing = wrapAngle360((float)_model.state.gps.directionToHome);
  if(!_failsafeRescueActive)
  {
    _failsafeRescueActive = true;
    _failsafeRescuePitchFlipped = false;
    _failsafeRescueBearing = homeBearing;
    _failsafeRescuePitchSign = -1.0f;
  }
  else
  {
    _failsafeRescueBearing = blendAngle(_failsafeRescueBearing, homeBearing, 0.2f);
  }

  const float insYaw = wrapAngle360(Utils::toDeg(-_model.state.attitude.euler[AXIS_YAW]));
  const float groundSpeed = std::max(_model.state.gps.velocity.raw.groundSpeed * 0.001f, 0.0f);
  const float course = groundSpeed > FAILSAFE_RESCUE_MIN_SPEED ? gpsCourseDegrees(_model) : _failsafeRescueBearing;
  const float courseError = wrapAngle180(_failsafeRescueBearing - course);
  const float crabAngle = groundSpeed > FAILSAFE_RESCUE_MIN_SPEED
    ? Utils::clamp(courseError * 0.35f, -FAILSAFE_RESCUE_MAX_CRAB_ANGLE, FAILSAFE_RESCUE_MAX_CRAB_ANGLE)
    : 0.0f;
  const float targetBearing = wrapAngle360(_failsafeRescueBearing + crabAngle);
  const float yawError = wrapAngle180(targetBearing - insYaw);
  const float closingSpeed = groundSpeed * std::cos(Utils::toRad(courseError));

  if(!_failsafeRescuePitchFlipped && groundSpeed > FAILSAFE_RESCUE_MIN_SPEED && std::abs(yawError) < 35.0f && closingSpeed < -0.25f)
  {
    _failsafeRescuePitchSign = -_failsafeRescuePitchSign;
    _failsafeRescuePitchFlipped = true;
  }

  const float alignFactor = Utils::clamp(1.0f - std::abs(yawError) / 90.0f, 0.0f, 1.0f);
  const float angleLimit = std::max((float)_model.config.level.angleLimit, 1.0f);
  const float maxAngle = Utils::clamp(std::min((float)_model.config.gps.rescueMaxAngle, angleLimit), 5.0f, angleLimit);
  const float targetSpeed = (float)std::max((int)_model.config.gps.rescueGroundSpeed, 1) * alignFactor;
  float pitchAngle = Utils::clamp((targetSpeed - std::max(closingSpeed, 0.0f)) * FAILSAFE_RESCUE_PITCH_GAIN, 0.0f, maxAngle);

  const float brakeDistance = std::max((float)_model.config.gps.rescueMinDistance * 2.0f, 1.0f);
  if(_model.state.gps.distanceToHome < brakeDistance)
  {
    pitchAngle *= Utils::clamp(_model.state.gps.distanceToHome / brakeDistance, 0.0f, 1.0f);
  }

  const float pitchCommand = _failsafeRescuePitchSign * Utils::clamp(pitchAngle / angleLimit, 0.0f, 1.0f);
  const float yawCommand = Utils::clamp(yawError * FAILSAFE_RESCUE_YAW_GAIN, -1.0f, 1.0f);

  float thrustCommandUs = Utils::clamp((int)_model.config.failsafe.throttle, (int)PWM_RANGE_MIN, (int)PWM_RANGE_MAX);
  if(canUseFailsafeLanding())
  {
    const float targetAltitude = std::max((float)_model.config.gps.rescueAltitude, _model.state.altitude.height);
    const float altitudeError = targetAltitude - _model.state.altitude.height;
    const float verticalRate = Utils::clamp(altitudeError * 0.35f, -0.75f, 1.5f);
    thrustCommandUs = normalizedToPwm(altHoldRateToInput(verticalRate));
  }

  setInput(AXIS_ROLL, PWM_RANGE_MID, true, true);
  setInput(AXIS_PITCH, pitchCommand * 500.0f + PWM_RANGE_MID, true, true);
  setInput(AXIS_YAW, yawCommand * 500.0f + PWM_RANGE_MID, true, true);
  setInput(AXIS_THRUST, thrustCommandUs, true, true);

  if(_model.config.debug.mode == DEBUG_RTH || _model.config.debug.mode == DEBUG_GPS_RESCUE_THROTTLE_PID)
  {
    _model.state.debug[0] = std::clamp(lrintf(_model.state.gps.distanceToHome), -32000l, 32000l);
    _model.state.debug[1] = std::clamp(lrintf(targetBearing * 10.0f), -32000l, 32000l);
    _model.state.debug[2] = std::clamp(lrintf(yawError * 10.0f), -32000l, 32000l);
    _model.state.debug[3] = std::clamp(lrintf(closingSpeed * 100.0f), -32000l, 32000l);
    _model.state.debug[4] = std::clamp(lrintf(pitchAngle * 10.0f), -32000l, 32000l);
    _model.state.debug[5] = std::clamp(lrintf((thrustCommandUs - PWM_RANGE_MID) * 10.0f), -32000l, 32000l);
  }
}

void Input::finishFailsafeLanding(DisarmReason reason)
{
  _model.state.failsafe.phase = FC_FAILSAFE_LANDED;
  _model.state.failsafe.timeout = 0;
  resetFailsafeRescueState();
  _model.disarm(reason);
}

void Input::resetFailsafeRescueState()
{
  _failsafeRescueActive = false;
  _failsafeRescuePitchFlipped = false;
  _failsafeRescueBearing = 0.0f;
  _failsafeRescuePitchSign = -1.0f;
}

void FAST_CODE_ATTR Input::filterInputs(InputStatus status)
{
  Utils::Stats::Measure filterMeasure(_model.state.stats, COUNTER_INPUT_FILTER);
  uint32_t startTime = micros();

  const bool newFrame = status != INPUT_IDLE;
  const bool interpolation = _model.config.input.interpolationMode != INPUT_INTERPOLATION_OFF && _model.config.input.filterType == INPUT_INTERPOLATION;

  if(interpolation)
  {
    if(newFrame)
    {
      _step = 0.0f;
    }
    if(_step < 1.f)
    {
      _step += _model.state.input.interpolationStep;
    }
  }

  for(size_t c = 0; c < _model.state.input.channelCount; c++)
  {
    float v = _model.state.input.buffer[c];
    if(c <= AXIS_THRUST)
    {
      v = interpolation ? _interpolate(_model.state.input.bufferPrevious[c], v, _step) : v;
    }
    setInput((Axis)c, v, newFrame);
  }

  if(_model.config.debug.mode == DEBUG_RX_TIMING)
  {
    _model.state.debug[3] = micros() - startTime;
  }
}

void FAST_CODE_ATTR Input::updateFrameRate()
{
  const uint32_t now = micros();
  const uint32_t frameDelta = now - _model.state.input.frameTime;

  _model.state.input.frameTime = now;
  _model.state.input.frameDelta += (((int)frameDelta - (int)_model.state.input.frameDelta) >> 3); // avg * 0.125
  _model.state.input.frameRate = 1000000ul / _model.state.input.frameDelta;

  if (_model.config.input.interpolationMode == INPUT_INTERPOLATION_AUTO && _model.config.input.filterType == INPUT_INTERPOLATION)
  {
    _model.state.input.interpolationDelta = Utils::clamp(_model.state.input.frameDelta, (uint32_t)4000, (uint32_t)40000) * 0.000001f; // estimate real interval
    _model.state.input.interpolationStep = _model.state.loopTimer.intervalf / _model.state.input.interpolationDelta;
  }

  if(_model.config.debug.mode == DEBUG_RC_SMOOTHING_RATE)
  {
    _model.state.debug[0] = _model.state.input.frameDelta / 10;
    _model.state.debug[1] = _model.state.input.frameRate;
  }

  // auto cutoff input freq
  float freq = std::max(_model.state.input.frameRate * _model.state.input.autoFactor, 15.f); // no lower than 15Hz
  if(freq > _model.state.input.autoFreq * 1.1f || freq < _model.state.input.autoFreq * 0.9f)
  {
    _model.state.input.autoFreq += 0.25f * (freq - _model.state.input.autoFreq);
    if(_model.config.debug.mode == DEBUG_RC_SMOOTHING_RATE)
    {
      _model.state.debug[2] = lrintf(freq);
      _model.state.debug[3] = lrintf(_model.state.input.autoFreq);
    }
    FilterConfig conf((FilterType)_model.config.input.filter.type, _model.state.input.autoFreq);
    FilterConfig confDerivative((FilterType)_model.config.input.filterDerivative.type, _model.state.input.autoFreq);
    for(size_t i = 0; i < AXIS_COUNT_RPYT; i++)
    {
      if(_model.config.input.filter.freq == 0)
      {
        _model.state.input.filter[i].reconfigure(conf, _model.state.loopTimer.rate);
      }
      if(_model.config.input.filterDerivative.freq == 0)
      {
        _model.state.innerPid[i].ftermFilter.reconfigure(confDerivative, _model.state.loopTimer.rate);
      }
    }
  }

  if(_model.config.debug.mode == DEBUG_RX_TIMING)
  {
    _model.state.debug[1] = micros() - now;
  }
}

Device::InputDevice * Input::getInputDevice()
{
  Device::SerialDevice * serial = _model.getSerialStream(SERIAL_FUNCTION_RX_SERIAL);
  if(serial && _model.isFeatureActive(FEATURE_RX_SERIAL))
  {
    switch(_model.config.input.serialRxProvider)
    {
      case SERIALRX_IBUS:
        _ibus.begin(serial);
        _model.logger.info().logln(F("RX IBUS"));
        return &_ibus;

      case SERIALRX_SBUS:
        _sbus.begin(serial);
        _model.logger.info().logln(F("RX SBUS"));
        return &_sbus;

      case SERIALRX_CRSF:
        _crsf.begin(serial, _model.isFeatureActive(FEATURE_TELEMETRY) ? &_telemetry : nullptr);
        _model.logger.info().logln(F("RX CRSF"));
        return &_crsf;
    }
  }
  else if(_model.isFeatureActive(FEATURE_RX_PPM) && _model.config.pin[PIN_INPUT_RX] != -1)
  {
    _ppm.begin(_model.config.pin[PIN_INPUT_RX], _model.config.input.ppmMode);
    _model.logger.info().log(F("RX PPM")).log(_model.config.pin[PIN_INPUT_RX]).logln(_model.config.input.ppmMode);
    return &_ppm;
  }
#if defined(ESPFC_ESPNOW)
  else if(_model.isFeatureActive(FEATURE_RX_SPI))
  {
    int status = _espnow.begin();
    _model.logger.info().log(F("RX ESPNOW")).logln(status);
    return &_espnow;
  }
#endif

  return nullptr;
}

}
