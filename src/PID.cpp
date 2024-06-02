MIT License

Copyright(c) 2024 Jamal Meizongo

    Permission is hereby granted,
    free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

    The above copyright notice and this permission notice shall be included in all copies
    or
    substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS",
    WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
    DAMAGES OR OTHER
    LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.

#include <Arduino.h>
#include "PID.h"

    PID::PID()
{
}

PID::PID(float _Kp, float _Ki, float _Kd)
{
    Initialize(_Kp, _Ki, _Kd);
}

// A call to an empty constructor should be followed by a call to the Initiailize() function to set the gain values
void PID::Initialize(float _Kp, float _Ki, float _Kd)
{
    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;
    ResetPID();
}

// Resets PID
void PID::ResetPID(void)
{
    previousError = 0;
    integral = 0;
    lastTime = millis();
}

// Main function to be called to get PID control value
int PID::Compute(float currentError)
{
    unsigned long currentTime = millis();
    double interval = (currentTime - lastTime) / 1000.0;
    float proportional = currentError;
    integral = integral + (currentError * interval);
    double derivative = (currentError - previousError) / interval;
    previousError = currentError;
    lastTime = currentTime;
    return ((Kp * proportional) + (Ki * integral) + (Kd * derivative));
}