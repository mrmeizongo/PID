MIT License

### Copyright (c) 2024 Jamal Meizongo

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

This is a simple PID controller I implemented for use in my RC plane flight stabilization software. Inspiration for this came from a great video by DigiKey about PID controllers on youtube. You can check out the video [here](https://www.youtube.com/watch?v=tFVAaUcOm4I).

You might have to implement a way to mitigate against integral windup depending on what you're controlling.
A simple way to achieve this is to reset the integral variable of the PID controller. You can use the Reset() function to achieve this.

Below are other slightly more sophisticated ways to deal with integral windup:

#### NOTE: You will have to modify the PID library to achieve these.

1. Initializing the Controller Integral: You can initialize the controller integral to a desired value, for instance, to the value before the problem.

2. Increasing the Setpoint in a Suitable Ramp: Gradually increasing the setpoint can help prevent sudden large errors and thus reduce the chance of integral windup.

3. Conditional Integration: This involves disabling the integral function until the process variable (PV) has entered the controllable region.

4. Anti-Windup Schemes: These are methods that prevent the integral term from accumulating beyond certain limits. One common method is the back-calculation anti-windup, which uses a feedback loop to unwind the PID Controller block internal integrator when the controller hits specified saturation limits
