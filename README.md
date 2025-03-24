# ST-MPC-FL-CAR
Tracking controller for a car-like vehicle based on the joint use of feedback linearization and set-theoretic MPC. Code developed for a book chapter submission entitled "A Feedback-Linearized Model Predictive
Control Strategy for Constrained Wheeled Mobile Robots"

Authors: Cristian Tiriolo and Walter Lucia, Concordia Unviersity (Canada).

# Considered model 
Kinematic model (bicycle model) of an input-constrained car-like vehicle

# Prerequisites 
The code was tested on Matlab 2022a environment. It requires the Ellipsoidal Toolbox ET  (https://www.mathworks.com/matlabcentral/fileexchange/21936-ellipsoidal-toolbox-et). 

# Main File for simulating the controller (DEMO)
- ST_FL_Trajectory_Tracking_Car.m: It's the main script that simulates the Trajectory tracking controller combining Feedback Linearization and Set-Theoretic strategy.
