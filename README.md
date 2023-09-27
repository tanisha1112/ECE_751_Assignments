### Kalman Filter Simulation
Software Project for the course ECE 751 Detection and Estimation Theory. 

The project is divided into 2 parts:
ECE_751_proj1 and ECE_751_proj2. 

The project1 uses the benchmark example of the velocity motion model to create the function:
function [X,Y] = kalman_filter(N,F,G,C,sigma_w,sigma_a, X_0)

Project2 includes the benchmark case where we are using the Kalman filter to estimate the position and velocity of a moving object based on noisy measurements of its position.
Kalman filter will be used to make future predictions using past values to correct the estimated state.
