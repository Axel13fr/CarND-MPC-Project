# Model Predictive Control Project

Overview
---
This repository contains the code needed to build a program which connects to the [Udacity Simulator](https://github.com/udacity/self-driving-car-sim/releases) and drives the vehicle using a model predictive control approach.

The Project
---
The goals / steps of this project are the following:
* Optimize PID parameters to have the vehicle stay on track and drive as fast as possible safely 
* Summarize the results and document how parameters were tuned


## Tuning the PID Parameters

I approached this problem by going over 2 steps:
1. Tuning the PID manually until I had something where the vehicle could drive a full lap
2. Use an optimization algorithm (in my case twiddle) to come to a more finely optimized result 

### The effect of P, I and D parameters

P causes the controler to response proportionnaly to the error. Its main influence can be seen as the responsiveness of the controller to an error. If the error increases very fast as in a sharp turn, P must be high enough to provide enough reactivity. Too much reactivity will cause an overshoot, causing undesired oscillations.

D is proportionnal to the error derivative and will be useful to attenuate an overreacting filter when getting closer to the setpoint, for example to recude overshoot effects of a high P.

I is proportionnal to the integral of the error. It is used to account for any given bias in the control. For example when a 0 steering angle would not have the vehicle's wheels be aligned with the vehicle axis, the vehicle will always go a bit left or right instead of straight. By integrating this error over time, the I coefficient will correct that. 

### Manually tuning the parameters

I first started with tuning the P parameter. I increased it until I had a control reactive enough to cover for the first turn of the track but not oscilliating too much while still overshooting.

To reduce overshoot, I gradually increased D until it proved more stable but not too much as it would cause in growing oscilliations up to  potentially divergence. At that point, the vehicle would drive well until the last turn which is quite sharp. To cope against that, I had to increase again the P parameter (now possible with a higher D) and in turn adjust D as well.

Finally, I introduced a pinch of I, the effect of which is hard to check visually. I set it to an initial value to prepare for a finer automatic optimization.

### Finely optimizing with Twiddle

To get better performances, I implented twiddle to automatically search for better parameters based on the accumulated squared cross track error, over a fixed number of steps.

First, I started with my manually tuned values for Kp, Ki, Kd: 0.8, 0.0004, 4.5.
I ran twiddle over the first 500 steps of the track to optimize the PID when going mostly straight.
I got as a result for Kp, Ki, Kd: 0.860978 0.000360201 4.14125.

Second, when testing this result on the full track, I had large overshoots on the final sharp turn, causing the vehicle to dangerously go off track. To compensate for that, I had to take this part of the lap into account so I ran twiddle over the first 2700 steps to include that sharp turn.  
I got as a result for Kp, Ki, Kd: 0.666776,0.000689087,4.25935. As one can see, the final optimization reduced P for less overshoot and increased I and D up to a certain point.


### Final result

The car can stay on track for the complete lap but controlling it only using the current cross track error has shown its limit:
the PID provides a reactive response withouth any possible anticipation. For a continuous sharp turn, this approach as shown its limit as the vehicle doesn't follow smoothly the middle of the track and keeps on overshooting.

Let's see what model predictive control brings :)

