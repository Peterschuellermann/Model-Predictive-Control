# Model Predictive Controller
This is an MPC which can control an autonomous race car around a racetrack. 
You can see it in action [here](https://youtu.be/C9X5vYnxYKs).

## The Model
The first step of the calculations was to change the vehicles coordinate references to match the global references. I used this code for it: 
```cpp
for (int i = 0; i \< ptsx.size(); i
  double shiftx = ptsx[i](#) - px;
  double shifty = ptsy[i](#) - py;

  ptsx[i](#) = (shiftx * cos(0 - psi) - shifty * sin(0 - psi));
  ptsy[i](#) = (shiftx * sin(0 - psi) + shifty * cos(0 - psi));
} 
```

### Boundaries 
To successfully steer the racer around the track I used boundaries to define its available parameters.

```cpp
// set all non-actuators upper and lower limits
// to the max negative and positive values.
for (int j = 0; j \< delta_start; j  vars_lowerbound[j](#) = -1.0e19;
  vars_upperbound[j](#) = 1.0e19;
}


// the upper and lower limits of delta are ser tp -25 and 25
// degrees (values are in radians)
for (int k = delta_start; k \< a_start; k  vars_lowerbound[k](#) = -0.436332 * Lf;
  vars_upperbound[k](#) = 0.436332 * Lf;
}


// Acceleration and decceleration upper and lower limits.
for (int l = a_start; l \< n_vars; l  vars_lowerbound[l](#) = -1.0;
  vars_upperbound[l](#) = 1.0;
}
```

### Cost Function 
to optimize the trajectory of the vehicle I used a cost function that would rate any trajectory on their cross track error, steering error, velocity, steering angle, acceleration, change in steering angle, and change in throttle.
I used the following weights to those measures: 
* cross track error = 2000
* steering error = 2000
* velocity = 1
* steering angle = 10
* acceleration = 10
* change in steering angle = 150
* change in throttle = 15

This resulted in a smooth driving experience that would drive securely at a reference velocity of 80% and a neck-breaking experience at 100% throttle.
I, therefore, performed most of my experiments at 80% throttle.

```cpp
for (int i = 0; i \< N; i  fg[0](#) += 2000 * CppADpow(vars[cte_start + i](#) - ref_cte, 2);
  fg[0](#) += 2000 * CppADpow(vars[epsi_start + i](#) - ref_epsi, 2);
  fg[0](#) += 1*CppADpow(vars[v_start + i](#) - ref_v, 2);
}
for (int j = 0; j \< N - 1; j  fg[0](#) += 10 * CppADpow(vars[delta_start + j](#), 2);
  fg[0](#) += 10 * CppADpow(vars[a_start + j](#), 2);
}
for (int k = 0; k \< N - 2; k  fg[0](#) += 150 * CppADpow(vars[delta_start + k + 1](#) - vars[delta_start + k](#), 2);
  fg[0](#) += 15 * CppADpow(vars[a_start + k + 1](#) - vars[a_start + k](#), 2);
} 
```

### Actuators
For each of the ten predicted steps, I would calculate optimal steering, and acceleration parameters to follow the trajectory. Even though I would only use the first step and scrap the other nine, this would help the vehicle to anticipate curves and brake not only when the apex of the curve was reached, like the PID controller, but already up to one second beforehand. 

Each time step would be predicted using the following formula:
```cpp
AD\<double\> f0 = coeffs[0](#) + coeffs[1](#) * x0 + coeffs[2](#) * x0 * x0 + coeffs[3](#) * x0 * x0 * x0;
AD\<double\> psides0 = CppADatan(3 * coeffs[3](#) * x0 * x0 + 2 * coeffs[2](#) * x0 + coeffs[1](#));

fg[2 + x_start + l](#) = x1 - (x0 + v0 * CppADcos(psi0) * dt);
fg[2 + y_start + l](#) = y1 - (y0 + v0 * CppADsin(psi0) * dt);
fg[2 + psi_start + l](#) = psi1 - (psi0 - v0 * delta0 / Lf * dt);
fg[2 + v_start + l](#) = v1 - (v0 + a0 * dt);
fg[2 + cte_start + l](#) = cte1 - ((f0 - y0) + (v0 * CppADsin(epsi0) * dt));
fg[2 + epsi_start + l](#) = epsi1 - ((psi0 - psides0) - v0 * delta0 / Lf * dt);
```

## Timestep Length and Elapsed Duration (N & dt)

I used ten time steps, each 100ms apart from each other. This were the first values I experimented with, and they proved sufficient. I tried 25 steps, and I tried a spacing of 200ms, but they either cost too much performance or caused the vehicle to jitter across the track.

## Model Predictive Control with Latency

To work with the imposed latency of 100ms, I used the bike vehicle model to calculate an approximation of where the vehicle would be at that time. I then used these assumptions to calculate the required actuator settings for that moment in time and passed them on to the simulator.
My code looked like this.

```cpp
const double dt = 0.1; // the latency will be 100ms

// calculate the new assumed state in dt time.
const double x_dt = 0.0 + v * dt;
const double y_dt = 0.0;
const double psi_dt = 0.0 + v * (-delta) / Lf * dt;
const double v_dt = v + a * dt;
const double cte_dt = cte + v * sin(epsi) * dt;
const double epsi_dt = epsi + v * (-delta)/Lf * dt;


EigenVectorXd state(6);

// use the new state as basis for all further calculations
state \<\< x_dt, y_dt, psi_dt, v_dt, cte_dt, epsi_dt;


//generate the trajectory
auto vars = mpc.Solve(state, coeffs);
```

## Dependencies

* cmake \>= 3.5
 * All OSes: [click here for installation instructions](#)(https://cmake.org/install/)
* make \>= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](#)(https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](#)(http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g  * Linux: gcc / g  * Mac: same deal as make - [install Xcode command line tools](#)((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](#)(http://www.mingw.org/)
* [uWebSockets](#)(https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
	```
	`git clone https://github.com/uWebSockets/uWebSockets
	cd uWebSockets
	git checkout e94b6e1
	```
	`Some function signatures have changed in v0.14.x. See [this PR](#)(https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](#)(https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](#)(http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](#)(https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](#)(./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Code Style

Please (do your best to) stick to [Google's C](#)(https://google.github.io/styleguide/cppguide.html).
