#include "R503.hpp"

R503 r503 = R503(GPIO_NUM_26, GPIO_NUM_27, 0xFFFFFFFF, 0x00000000, 57600, &Serial1);

void setup()
{
    Serial.begin(115200);
    
    Serial1.setPins(gpioNumberToDigitalPin(GPIO_NUM_26), gpioNumberToDigitalPin(GPIO_NUM_27));
    

    auto r = r503.init();
    if(r == R503_SUCCESS) {
        Serial.println("R503 connected with success");
    } else {
        Serial.println("R503 not connected: Return code: " + String(r));
    }
}

void loop()
{
    Serial.println("Foi um loop. Next em 1 seg");
    delay(1000);
}