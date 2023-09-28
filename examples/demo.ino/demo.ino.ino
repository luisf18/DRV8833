#include "DRV8833.h"

// DRV8833 motor( 13, 14, 27, 4 ); // VESPA
DRV8833 motor( 12,13,4,5 ); // TREVINHO 0

void setup() {
  motor.begin();
  motor.bip(5,200,2000);
}

void loop() {
  motor.move( 1023, 1023 );
  Serial.printf("speed_0: %d speed_1: %d\n", motor.read(0), motor.read(1) );
  motor.bip(1,200,2000);
  delay(1000);
  motor.move( -1023, -1023 );
  Serial.printf("speed_0: %d speed_1: %d\n", motor.read(0), motor.read(1) );
  delay(1000);
  motor.stop();
  Serial.printf("speed_0: %d speed_1: %d\n", motor.read(0), motor.read(1) );
  delay(1000);
}
