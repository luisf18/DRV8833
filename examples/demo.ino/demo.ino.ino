#include "DRV8833.h"

// DRV8833 motor( 13, 14, 27, 4 ); // VESPA
DRV8833 motor( 12,13,4,5 ); // TREVINHO 0

void setup() {
  motor.begin();
  motor.bip(5,200,2000);
}

void loop() {
  motor.move( 1023, 1023 );
  delay(1000);
  motor.move( -1023, -1023 );
  delay(1000);
  motor.stop();
  delay(1000);
}
