#pragma once

#include "DiyActivePedal_types.h"


static const long ABS_ACTIVE_TIME_PER_TRIGGER_MILLIS = 100;
static const long RPM_ACTIVE_TIME_PER_TRIGGER_MILLIS = 100;
static int RPM_VALUE_LAST = 0;

class ABSOscillation {
private:
  long _timeLastTriggerMillis;
  long _absTimeMillis;
  long _lastCallTimeMillis = 0;

public:
  ABSOscillation()
    : _timeLastTriggerMillis(0)
  {}

public:
  void trigger() {
    _timeLastTriggerMillis = millis();
  }
  
  float forceOffset(DAP_calculationVariables_st* calcVars_st) {


    long timeNowMillis = millis();
    float timeSinceTrigger = (timeNowMillis - _timeLastTriggerMillis);
    float absForceOffset = 0;

    if (timeSinceTrigger > ABS_ACTIVE_TIME_PER_TRIGGER_MILLIS)
    {
      _absTimeMillis = 0;
      absForceOffset = 0;
    }
    else
    {
      _absTimeMillis += timeNowMillis - _lastCallTimeMillis;
      float absTimeSeconds = _absTimeMillis / 1000.0f;
      absForceOffset = calcVars_st->absAmplitude * sin(calcVars_st->absFrequency * absTimeSeconds);
    }

    _lastCallTimeMillis = timeNowMillis;

    return absForceOffset;
    

  }
};

class RPMOscillation {
private:
  long _timeLastTriggerMillis;
  long _RPMTimeMillis;
  long _lastCallTimeMillis = 0;
  

public:
  RPMOscillation()
    : _timeLastTriggerMillis(0)
  {}
  float RPM_value =0;
  int32_t RPM_position_offset = 0;
public:
  void trigger() {
    _timeLastTriggerMillis = millis();
  }
  
  void forceOffset(DAP_calculationVariables_st* calcVars_st) {


    long timeNowMillis = millis();
    float timeSinceTrigger = (timeNowMillis - _timeLastTriggerMillis);
    float RPMForceOffset = 0;
    float RPM_max_freq = calcVars_st->RPM_max_freq;
    float RPM_min_freq = calcVars_st->RPM_min_freq;
    //float RPM_max =10;
    float RPM_amp = calcVars_st->RPM_AMP;
    if(RPM_value==0)
    {
      RPM_min_freq=0;
    }


    float RPM_freq=constrain(RPM_value*(RPM_max_freq-RPM_min_freq)/100, RPM_min_freq, RPM_max_freq);
    


    if (timeSinceTrigger > RPM_ACTIVE_TIME_PER_TRIGGER_MILLIS)
    {
      _RPMTimeMillis = 0;
      RPMForceOffset = RPM_VALUE_LAST;
    }
    else
    {
      _RPMTimeMillis += timeNowMillis - _lastCallTimeMillis;
      float RPMTimeSeconds = _RPMTimeMillis / 1000.0f;

      //RPMForceOffset = calcVars_st->absAmplitude * sin(calcVars_st->absFrequency * RPMTimeSeconds);
      RPMForceOffset = RPM_amp * sin( 2*PI* RPM_freq* RPMTimeSeconds);
    }

    _lastCallTimeMillis = timeNowMillis;
    RPM_VALUE_LAST=RPMForceOffset;
    RPM_position_offset = calcVars_st->stepperPosRange*(RPMForceOffset/calcVars_st->Force_Range);
    //return RPMForceOffset;
    

  }
};
