Kalman Filter simulation 
===========================

This project is based on a simulation of the Kalman filter. Using state vectors, a Kalman signal model estimates the state. An example of a roughly constant velocity model is used for benchmarking.

Three graphs are plotted with respect to the velocity and position of the motion body.
They indicate the noise, true and estimated values. 
The first graph indicates the 

Files
-----

The repository contains the source code (ece_751_proj.m) that contains the function kalman_filter. 
It contains a function kalman_filter_estimate which estimates the position, velocity and Klaman gain matrix and also the MSE values vector.

It also contains an output folder (output) that contains the output results of the Matlab file. 

Installation notes
--------------------

These files were developed and tested on R2022a. The products need to run all of the files are:

* MATLAB R2022a

