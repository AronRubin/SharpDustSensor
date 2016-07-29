
#if defined (SPARK)
#include "application.h"
#ifndef ADC_RESOLUTION
#define ADC_RESOLUTION 12
#endif
#else
#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#ifndef ADC_RESOLUTION
#define ADC_RESOLUTION 10
#endif
#endif

#include "SharpDustSensor.h"

#define VOLT_PER_MGM3 5.83 // Datasheet says 5 but community says 5.83
#define NO_DUST_VOLTAGE 0.1 // Datasheet says 0.9 but community says 0.1
#define ADC_VREF 3.3
#define MGM3_SCALE (ADC_VREF / (1 << ADC_RESOLUTION) / VOLT_PER_MGM3)
#define MGM3_OFFSET NO_DUST_VOLTAGE

SharpDustSensor::SharpDustSensor( int32_t sensorID, uint8_t ledPin, uint8_t readPin ) {
  _ledPin = ledPin;
  _readPin = readPin;
  _sensorID = sensorID;
  _useMultisample = true;
  _useMovingAverage = true;
  _pauseInterrupts = false;
  _lastTime = 0;
  _lastValue = 0;
}

bool SharpDustSensor::begin() {
  pinMode( _ledPin, OUTPUT );
}
void SharpDustSensor::setSamplesPerValue( uint8_t samples ) {
  _samples = samples;
}
uint8_t SharpDustSensor::getSamplesPerValue() const {
  return _samples;
}

void SharpDustSensor::setPauseInterrupts( bool pauseInterrupts ) {
  _pauseInterrupts = pauseInterrupts;
}
bool SharpDustSensor::getPauseInterrupts( bool pauseInterrupts ) const {
  return _pauseInterrupts;
}
void SharpDustSensor::setLedPin( uint8_t pin ) {
  _ledPin = pin;
}
uint8_t SharpDustSensor::getLedPin() const {
  return _ledPin;
}

void SharpDustSensor::setReadPin( uint8_t pin ) {
  _readPin = pin;
}
uint8_t SharpDustSensor::getReadPin() const {
  return _readPin;
}


int32_t SharpDustSensor::readRaw() {
  bool pauseInterrupts = _pauseInterrupts;
  bool isFirst = _lastTime == 0;
  int32_t reading = 0;
  long tnow = millis();

  if( _nextTime != 0 && _lastTime + 10 > tnow ) {
    return _lastValue;
  }

  if( pauseInterrupts ) {
    noInterrupts();
  }

  _lastTime = tnow;
  digitalWrite( _ledPin, LOW ); // go LOW to activate LED
  delayMicroseconds( 280 ); // enough time to rise
  if( _useMultisample ) {
    int32_t samples[3] = {
      analogRead( _readPin ),
      analogRead( _readPin ),
      analogRead( _readPin );
    };
    // sort to get the median
    if( samples[1] < samples[0] ) {
      reading = samples[0];
      samples[0] = samples[1];
      samples[1] = reading;
    }
    if( samples[1] > samples[2] ) {
      reading = samples[1];
      samples[1] = samples[2];
      samples[2] = reading;
    }
    reading = samples[1];
  } else {
    reading = analogRead( _readPin );
  }
  delayMicroseconds( 40 ); // delay because the datasheet says
  digitalWrite( _ledPin, HIGH ); // go HIGH to deactivate LED

  if( pauseInterrupts ) {
    interrupts();
  }

  if( _useMovingAverage && !isFirst ) {
    _lastValue = (5*_lastValue + 3*reading)/8;
  }
  _lastValue = reading;

  return reading;
}

float SharpDustSensor::getDensity() {
  int32_t reading = readRaw();
  if( reading <= ((1 << ADC_RESOLUTION) / ADC_VREF * VOLT_PER_MGM3 * NO_DUST_VOLTAGE) ) {
    return 0;
  } else {
    return ((float)reading) * MGM3_SCALE - MGM3_OFFSET;
  }
}

/* Adafruit_Sensor implementation */
bool SharpDustSensor::getEvent( sensors_event_t *event ) {
  if( !event ) {
    return false;
  }

  memset( event, 0, sizeof(*event) );

  float density = getDensity();
  if( _lastTime == 0 ) {
    return false;
  }
  event->version = sizeof(*event);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_DENSITY;
  event->timestamp = _lastTime;
  event->density = density;

  return true;
}

void  SharpDustSensor::getSensor( sensor_t *sensor ) {
  if( !sensor ) {
    return;
  }

  memset( sensor, 0, sizeof(*sensor) );
  strncpy( sensor->name, "GP2Y1010", sizeof(sensor->name) - 1 );
  sensor->name[sizeof(sensor->name) - 1] = '\0';
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_DENSITY;
  sensor->min_delay = 10; // Datasheet says 10ms cycle time
  sensor->max_value = 3.4 / VOLT_PER_MGM3; // Datasheet says 3.4 max Vo
  sensor->min_value = 0;
  sensor->resolution = MGM3_SCALE;
}
