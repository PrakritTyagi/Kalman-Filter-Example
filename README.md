# Kalman-Filter-Example

This program is based on a simple vehicle tracking problem in  x direction where its postion and velocity are to be estimated through kalman filter. This makes it a two dimention estimation and as a prerequisite, basic knowlwdge in linear algebra, probability and statiscs will be sufficient to understand EKF.

First i will describe the problem .

Consider a truck moving in 1-Dimensional space. The dynamic model is: System dynamics


where  x  and  v  are the position and velocity along the x axis.  atrue(t)  is the true acceleration at time  t  and  ω(t)  is the process noise. Accelerometer measurement model:


