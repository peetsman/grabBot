
#include <Arduino.h>

int myFunction(int x, int y) {
    return x + y;
}

void setup() {
    Serial.begin(9600);

    int result = myFunction(2, 3);

    Serial.print("The result is: ");
    Serial.println(result);
}

void loop() {
    // Nothing to repeat
}
