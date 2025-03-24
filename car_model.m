function [qdot] = car_model(t,q,v1,v2,l)

%This function implements the car kinematic model
%Giving the state q(k), the inputs v1(k) driving velocity, and v2(k) steering velocity, at time k, it computes the state at
%the next time step q(k+1) by solving the differential equations describing the kinematics of
%the robot.

x=q(1);
y=q(2);
theta=q(3);
phi=q(4);

q1dot=v1*cos(theta);
q2dot=v1*sin(theta);
q3dot = v1*tan(phi)/l;
q4dot = v2;

qdot= [q1dot;q2dot;q3dot;q4dot];
end