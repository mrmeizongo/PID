#include <Arduino.h>
#include "PID.h"

#define INPUT_PIN A0
#define OUTPUT_PIN D5

float Kp = 5.0f;
float Ki = 0.2f;
float Kd = 0.001f;
float IMax = 100.0f;

PID myController{_Kp, _Ki, _Kd, _iMax}; // Constructor with gain values set

float setPoint = 72;
float actual = 0;
float output = 0;

void setup()
{
    pinMode(INPUT_PIN, INPUT);
    pinMode(OUTPUT_PIN, OUTPUT);
}

void loop()
{
    actual = map(analogRead(INPUT_PIN), 0, 1024, 0, 255);

    float error = setPoint - actual;
    output = myController->Compute(error);
    analogWrite(OUTPUT_PIN, output);

    Serial.print("Output is: ");
    Serial.println(output);
}