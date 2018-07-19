## CarND-Controls-PID

The goal of this project is to implement a PID controller to control a vehicle around a lake race track. The steps of this project are following:

* Use the cross track error (CTE) and the velocity (mph) provided by simulator to computer the appropriate steering angle.
* Add another PID controller to control the speed, drive safely, as fast as possible.
* Implement Twiddle algorithm to optimize the parameters.

[//]: # (Image References)
[image1]: ./images/twiddle.png
[image2]: ./images/p.gif
[image3]: ./images/pd.gif
[image4]: ./images/speed.gif


![alt text][image4]

### Setup
* Download Simulator [here](https://github.com/udacity/self-driving-car-sim/releases)
* Install uWebSocketIO [here](https://github.com/uWebSockets/uWebSockets)

### Basic Build Instructions
1. Make a build directory: `mkdir build && cd build`
2. Compile: `cmake .. && make`
3. Run it: `./pid`. 

### Implement PID Controller

A PID controller is a control loop feedback mechanism. It continuously calculates an error value as the difference between a desired setpoint(SP) and a measured process variable (PV) and applies a correction based on proportional (P), integral (I), and derivative (D) terms.

#### Steer PID Controller

I first set all gains to 0. This means we are not doing any corrections to the errors. The car went off track immediately.

##### 1. P Controller
P controller applies corrections that are porportional to the cross track error. If the car is far to the right it steers hard to the left, if it's slightly to the right it steers slightly to the left.

```
steering_value -= Kp*CTE
```

I increased the P gain (Kp) and the car oscillated between left and right until it eventually went off track again. The larger the value of Kp, the harder the car pulls to left and right. This is the natural overshooting effect of P Controller. 

![alt text][image2]

##### 2. D Controller
To avoid the overshoot from P Controller, I added D Controller to form a PD Controller. The steering value is not just proportionally related to CTE, but also to the temporal derivative of CTE.

```
steering_value -= Kp*CTE - Kd*dCTE/dt
```

When the car turned enough to reduce the CTE, it will notice it's already reducing the error because the error is becoming smaller over time. The car counter steers.

I increased the D gain (Kd) until the oscillation went away. If Kd is too high, the car will slow down and jitter at the target position. I obtained good result with **Kp = 0.09 and Kd = 2.0**, the car successfully drive over a lap around the track.

![alt text][image3]

##### 3. I Controller
The I term considers all past values of the CTE and it's measured by the integral of CTE over time. We need it for the systematic bias over a long period of time that prevents the car to get to the desired trajectory. The I Controller works to cancel out this error.

```
steering_value -= Kp*CTE - Kd*dCTE/dt - Ki*sum(CTE)
```

I observed no obvious steering drift with the PD Controller, and left the I gain (Ki) to 0.0.


#### Speed PID Controller
To achieve a higher speed, I added a second PID Controller to control the throttle, and further optimized by relating the throttle value with the steering angle. With a minimum speed of 30 mph, I speed up when the steering angle is small, and I slow down when the steering angle is large.
 
I did this in lines 7 to 9, 52 to 54, and 85 to 90 in `main.cpp`.

![alt text][image4]

#### Parameter Optimization
I used twiddle algorithm to optimize the PID gains - Kp, Ki, and Kd.

Here's the pseudo code:
![alt text][image1]

I did this in lines 61 to 114 in the function `Twiddle()` of `PID.cpp`.

### Results
The vehicle safely drove around the track with 2 PID controllers: one for steering (Kp: 0.114690, Ki: 0.000000, Kd: 2.045754) and another for speed (Kp: 0.121535, Ki: 0.006990, Kd: 0.942458). The maximum speed achieved was 70 mph.