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

v<sub>t+1</sub>=v<sub>t</sub>+a<sub>t</sub>∗dt

#### Errors

The cross track error is the distance between the center of the road and the vehicle's position:

cte<sub>t+1</sub> =ctet +v<sub>t</sub>∗sin(eψ<sub>t</sub>)∗dt

The orientation error is angle difference between the vehicle orientation and the tangential angle of the path to follow (ψdes<sub>t</sub>) + the error caused by the vehicle movement:

eψ<sub>t+1</sub> =ψ<sub>t</sub> −ψdes<sub>t</sub> +(v<sub>t</sub>/L<sub>f</sub>∗δ<sub>t</sub>∗dt)

### Prediction window: timestep length and elapsed duration

### Polynomial Fitting and MPC Preprocessing

### Model Predictive Control with Latency


### Final result

The car can stay on track for the complete lap but controlling it only using the current cross track error has shown its limit:
the PID provides a reactive response withouth any possible anticipation. For a continuous sharp turn, this approach as shown its limit as the vehicle doesn't follow smoothly the middle of the track and keeps on overshooting.

Let's see what model predictive control brings :)

