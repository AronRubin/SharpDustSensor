
#include <SharpDustSensor/SharpDustSensor.h>

SharpDustSensor sensor = SharpDustSensor( 1, D1, A0 );

void setup() {
  Serial.begin( 9600 );
  sensor.begin();
}

void loop() {
  float density = sensor.getDensity();
  Serial.print( "density = " );
  Serial.println( density );
  delay( 1000 );
}
