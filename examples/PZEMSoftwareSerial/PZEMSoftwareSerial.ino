#include <PZEM017v1.h>

/* Use software serial for the PZEM
 * Pin 4 Rx (Connects to the Tx pin on the PZEM)
 * Pin 5 Tx (Connects to the Rx pin on the PZEM)
*/
PZEM017v1 pzem(4, 5); //WMOS D1 4=D1, 5=D2

void setup() {
  Serial.begin(115200);
}

void loop() {
    float voltage = pzem.voltage();
    if( !isnan(voltage) ){
        Serial.print("Voltage: "); Serial.print(voltage); Serial.println("V");
    } else {
        Serial.println("Error reading voltage");
    }

    float current = pzem.current();
    if( !isnan(current) ){
        Serial.print("Current: "); Serial.print(current); Serial.println("A");
    } else {
        Serial.println("Error reading current");
    }

    float power = pzem.power();
    if( !isnan(power) ){
        Serial.print("Power: "); Serial.print(power); Serial.println("W");
    } else {
        Serial.println("Error reading power");
    }

    float energy = pzem.energy();
    if( !isnan(energy) ){
        Serial.print("Energy: "); Serial.print(energy,3); Serial.println("kWh");
    } else {
        Serial.println("Error reading energy");
    }

    Serial.println();
    delay(2000);
}
