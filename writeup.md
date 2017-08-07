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

To have MPC running smoothly at higher speed, it was necessary to tune the costs a little bit. Safety first meant to prioritize a lower cte and orientation error over speed error: the optimizer can then find solutions with lower speeds while keeping the vehicle at the center of the path, which is what a human driver would do! In practice, the solution causes the vehicle to sometimes brake down to 30mph while still going 50mph when turns are not sharp.

To keep the driving as smooth as possible, I increased the error generated by large steering value and accelerations differences. This incentive works well to keep the ride virtually confortable !

### Polynomial Fitting and MPC Preprocessing

To make computations easier (specially for cte and epsi) for the solver as well as to be able to draw trajectories, waypoints are first converted from world to vehicle coordinates.

A polynomial of order 3 is fitted to the waypoints and from there, the initial state is calculated and then fed to the solver. 

### Model Predictive Control with Latency

I optimized my model for latency over 2 approaches:
- tuning dt to force the optimizer to consider more time between 2 actuator commands (see above) : plan a solution with a higher response time
- predicting the current position to Latency in the future as the initial state for the optimizer

The second approach specifically allows the optimizer to find a trajectory which will compensate for the initial delay.


### Final result

The car can stay on track and drive smoothly all along at 50mph which is huge progress compared to the PID controller. It drives safely and adjust the speed approriately on sharp turns.

Further tuning in the costs could probably take it to 60+mph, as well as eliminating some rare occurence of rather "ugly" unsmooth trajectory solutions of the solver. The settings could also be adpated to difference speeds or situation to have a more ideal model for a real application. A real application requies as well more work on slip angle, ration and tires models to take these effects into account for better predictions.



