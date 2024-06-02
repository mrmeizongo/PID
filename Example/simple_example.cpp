#include <Arduino.h>
#include "PID.h"

#define INPUT_PIN A0
#define OUTPUT_PIN D5

float Kp = 5;
float Ki = 0.2;
float Kd = 0.001;

PID myController; // Empty constructor
// PID myController{Kp, Ki, Kd};    // Constructor with gain values set

float setPoint = 72;
float actual = 0;
float output = 0;

void setup()
{
    /*
     * NOTE: When an empty constructor is used to declare a controller, the Initialize() function must be called
     * to set the gain values and setup the controller
     */
    myController.Initialize(Kp, Ki, Kd);
    pinMode(INPUT_PIN, INPUT);
    pinMode(OUTPUT_PIN, OUTPUT);
}

void loop()
{
    actual = map(analogRead(INPUT_PIN), 0, 1024, 0, 255);
    actual = constrain(actual, 0, 255);

    float error = setPoint - actual;
    output = myController.Compute(error);
    analogWrite(OUTPUT_PIN, output);

    Serial.print("Output is: ");
    Serial.println(output);
}