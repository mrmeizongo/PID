/*
MIT License

Copyright(c) 2024 Jamal Meizongo

    Permission is hereby granted,
    free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

    The above copyright notice and this permission notice shall be included in all copies
    or
    substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS",
    WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
    DAMAGES OR OTHER
    LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#include <Arduino.h>
#include "PID.h"

PID::PID()
    : Kp{1}, Ki{1}, Kd{1}, IMax{100}
{
    // set previousDerivative as invalid on startup
    previousDerivative = NAN;
}

PID::PID(float _Kp, float _Ki, float _Kd, float _IMax)
    : Kp{_Kp}, Ki{_Ki}, Kd{_Kd}, IMax{_IMax}
{
    // set previousDerivative as invalid on startup
    previousDerivative = NAN;
}

// Resets PID
void PID::ResetI(void)
{
    integrator = 0;
    previousDerivative = NAN;
}

// Main function to be called to get PID control value
float PID::Compute(float currentError)
{
    unsigned long currentTime = millis();
    unsigned long dt = currentTime - previousTime;
    float output = 0.0f;
    float deltaTime;

    // if this PID hasn't been used for a full second then zero
    // the integrator term. This prevents I buildup from a
    // previous fight mode from causing a massive return before
    // the integrator gets a chance to correct itself
    if (previousTime == 0 || dt > 1000)
    {
        dt = 0;
        ResetI();
    }

    deltaTime = (float)dt * 0.001f;
    // Save last time Compute was run
    previousTime = currentTime;
    // Compute proportional component
    output += currentError * Kp;

    // Compute integral component if time has elapsed
    if ((fabsf(Ki) > 0) && (dt > 0))
    {
        integrator += (currentError * Ki) * deltaTime;
        // Limit integrator wind up
        if (integrator < -IMax)
        {
            integrator = -IMax;
        }
        else if (integrator > IMax)
        {
            integrator = IMax;
        }
        output += integrator;
    }

    // Compute derivative component if time has elapsed
    if ((fabsf(Kd) > 0) && (dt > 0))
    {
        float derivative;

        if (isnanf(previousDerivative))
        {
            // Reset called. Suppress first derivative term
            // as we don't want a sudden change in input to cause
            // a large D output change
            derivative = 0;
            previousDerivative = 0;
        }
        else
            derivative = (currentError - previousError) / deltaTime;

        // discrete low pass filter, cuts out the
        // high frequency noise that can drive the controller crazy
        float RC = 1 / (2 * M_PI * fCut);
        derivative = previousDerivative +
                     ((deltaTime / (RC + deltaTime)) *
                      (derivative - previousDerivative));

        // Update state
        previousError = currentError;
        previousDerivative = derivative;

        // Add in derivative component
        output += derivative * Kd;
    }

    return output;
}