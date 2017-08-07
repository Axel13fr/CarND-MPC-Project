# Model Predictive Control Project

Overview
---
This repository contains the code needed to build a program which connects to the [Udacity Simulator](https://github.com/udacity/self-driving-car-sim/releases) and drives the vehicle using a model predictive control approach.

The Project
---
The goals / steps of this project are the following:
* Implement a predictive model to have the vehicle stay on track and drive as fast as possible safely 
* Summarize the results and document how the model was constructured and the parameters were tuned


## From PID to Model Predictive control 

The previous PID controller project was implemented as directly reacting to the cross track error (CTE). It prevented from taking advantage of the 2 following informations :
1. The future position, speed and orientation can be approximated using a dynamic model, hence the future CTE can be estimated as well
2. The future expected position is known because the precise route to follow is planned before hand

These 2 informations allows the problem to be forumulated as an optimization problem: find acceleration and steering wheel setpoints such that the position/orientation/speed error is minimized by looking at several positions in the future.

## Model predictive control implementation

### The model

#### Kinematic model

The kinematic model used to predict how the car moves is defined as follow (position, orientation ψ, speed v and acceleration a):

x<sub>t+1</sub> =x<sub>t</sub> +v<sub>t</sub> ∗cos(ψ<sub>t</sub>)∗dt

y<sub>t+1</sub> =y<sub>t</sub>+v<sub>t</sub>∗sin(ψ<sub>t</sub>)∗dt

ψ<sub>t+1</sub> =ψ<sub>t</sub>+Lfv<sub>t</sub>∗δ∗dt

v<sub>t+1</sub> =v<sub>t</sub>+a<sub>t</sub>∗dt

#### Errors

The cross track error is the distance between the center of the road and the vehicle's position:

cte<sub>t+1</sub> =cte<sub>t</sub> +v<sub>t</sub>∗sin(eψ<sub>t</sub>)∗dt

The orientation error is angle difference between the vehicle orientation and the tangential angle of the path to follow (ψdes<sub>t</sub>) + the error caused by the vehicle movement:

eψ<sub>t+1</sub> =ψ<sub>t</sub> −ψdes<sub>t</sub> +(v<sub>t</sub>/L<sub>f</sub>∗δ<sub>t</sub>∗dt)

#### State and actuators  

The kinematic model and the errors form together the state vector: [x y ψ v cte eψ]

The actuator commands are the acceleration a and the steering wheel angle delta.

#### Optimization setup

The goal of the optimization is to choose actuator commands such that cte and eψ are minimized. For this purpose, we need to give constraints to the optimizer so that the kinematic model is respected (ex: the car can't teleport to the middle of the road). 

Only doing this will not be enough to come to safe driving solutions ! We need to make sure as well that :
- the vehicle will not stop in the middel of the road by penalizing a speed too far from the target speed
- the vehicle will not drive in a hectic way by penalizing the use of actuators
- the vehicle will drive smoothly by penalizing large variations of throttle or steering

Last but not least, the optimizer must be told what the maximum and minimum values for actuators : +/-25deg steering and [-1+1] acceleration/brake. 

### MPC tuning

### Prediction window: timestep length and elapsed duration

The timestep length N controls the number of predicted states in the future we will optimize on. Each of these prediction are done after dt time is elapsed.

With these 2 parameters, we control : 
- how far in the future we try to predict the vehicle state : T = N * dt   
- how often we can apply an actuation for the optimizer : dt

Because the model is only an approximation, using predictions to optimize the trajectory only makes sense for 1 to 2 secs. After that, the horizon won't match the vehicle predictions or the polynomial fit will not approximate the trajectory properly if there are many turns within the horizon.
Increasing N means more computation time for the optimizer while at the same time, a large dt will make it difficult to find a good solution.

I started out using T=3secs and tried dt of 0.05, 0.1 and 0.2. This prediction horizon was too large, causing the optimizer to generate chaotic solutions. I reduced it to 1 sec with a 0.1 dt and it worked pretty fine when without any latency and at low speeds (20-30mph).

For the final solution, I incread T to 2secs to have better results on higher speed and I used a dt of 0.2secs to for the optimizer to find solutions which would give more time between 2 actuator commands: this helped to deal with the latency.

### Optimizer tuning

### Polynomial Fitting and MPC Preprocessing

### Model Predictive Control with Latency


### Final result

The car can stay on track for the complete lap but controlling it only using the current cross track error has shown its limit:
the PID provides a reactive response withouth any possible anticipation. For a continuous sharp turn, this approach as shown its limit as the vehicle doesn't follow smoothly the middle of the track and keeps on overshooting.

Let's see what model predictive control brings :)

