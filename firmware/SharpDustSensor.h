/***************************************************************************
  Library for the Sharp Dust Sensor GP2Y10.

  MIT license, all text above must be included in any redistribution
 ***************************************************************************/

#ifndef __SHARPDUSTSENSOR_H__
#define __SHARPDUSTSENSOR_H__

#if defined (SPARK)
#include "application.h"
#else
#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#endif

#include "Adafruit_Sensor.h"

class SharpDustSensor : public Adafruit_Sensor {
public:
  SharpDustSensor( int32_t sensorID = -1, uint8_t ledPin = D2, uint8_t readPin = A0 );

  bool begin();
  void setUseMultisample( bool useMultisample );
  bool getMultisample() const;
  void setUseMovingAverage( bool useMovingAverage );
  bool getMovingAverage() const;
  void setPauseInterrupts( bool pauseInterrupts );
  bool getPauseInterrupts( bool pauseInterrupts ) const;
  void setLedPin( uint8_t pin );
  void setReadPin( uint8_t pin );
  uint8_t getLedPin() const;
  uint8_t getReadPin() const;

  float getDensity();

  /* Adafruit_Sensor implementation */
  bool  getEvent( sensors_event_t *event );
  void  getSensor( sensor_t *sensor );

private:
  int32_t readRaw();

  uint8_t _ledPin;
  uint8_t _readPin;
  int32_t _sensorID;
  bool _useMultisample;
  bool _useMovingAverage;

  int32_t _lastValue;
  long _lastTime;
  bool _pauseInterrupts;
};

#endif
