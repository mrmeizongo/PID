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
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
    DAMAGES OR OTHER
    LIABILITY,
    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/
#pragma once
#include <inttypes.h>

class PID
{
public:
    PID();                           // Empty Constructor
    PID(float, float, float, float); // Constructor with initialization parameters
    void ResetI(void);               // Reset PID integrator
    float Compute(float);            // Generate the PID output to be added to the servo

    float getKp(void) { return Kp; }
    float getKi(void) { return Ki; }
    float getKd(void) { return Kd; }

    void setKp(float _Kp) { Kp = _Kp; }
    void setKi(float _Ki) { Ki = _Ki; }
    void setKd(float _Kd) { Kd = _Kd; }

private:
    float Kp;
    float Ki;
    float Kd;
    float IMax;

    /// Low pass filter cut frequency for derivative calculation.
    ///
    /// 20 Hz because anything over that is probably noise, see
    /// http://en.wikipedia.org/wiki/Low-pass_filter.
    ///
    static const uint8_t fCut = 20;

    float integrator;
    float previousError;
    float previousDerivative; // for low-pass filter calculation
    unsigned long previousTime;
};