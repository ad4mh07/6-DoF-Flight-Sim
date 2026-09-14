# 6 Degree of Freedom Flight Simulator

## 1: Overview
I have built a 6 degree of freedom flight simulator in Python. Taking a vehicle model and initial flight conditions, it plots the flight path, and other important visualisations, such as angle of attack and side slip, velocities, etc. The core of the model, flat_earth_eom.py also uses atmospheric data for various aerodynamic computations, namely the 1976 USSA atmospheric model.

## 2: The Project

### 2.1: Repository structure
```
6-DoF-Flight-Sim/
|
|-- flights/
|   |-- flight_config.py
|   |-- flight_paths_readable.py
|   |-- flight_paths_technical.py
|
|-- governing_equations/
|   |-- flat_earth_eom.py
|
|-- numerical_integrators/
|   |--numerical_integration_methods.py
|
|-- saved figures/
|   |--{saved figures}.png
|
|-- tools/
|   |-- interpolators
|
|-- vehicle_models/
|   |-- CESSNA_172.py
|   |-- spheres.py
|
|-- README.md
|-- main_program.py
```

### 2.2: Files overview 
flight_config.py- contains two functions that, respectively, convert the two styles of flight paths (initial conditions) into readable arrays for the simulation in main_program

flight_paths_readable.py- contains various initial conditions, including a freefall, banked turn and forwards motion in human-terms

flight_paths_technical.py- contains the same initial conditions, but in technical terms, much more like the expected array, with more variables available to alter

flat_earth_eom.py- contains the physics behind the entire program; ultimately lists the differential equations to be approximated, but also contains coordinate conversions, atmospheric and aerodynamic calculations.

numerical_integration_methods.py- contains the chosen integrator; currently this is a self-implemented version of the Forward Euler Method.

interpolators- contains a self-implemented linear interpolator used in the main program.

CESSNA_172.py- contains vehicle data (in a dictionary) about the classic Cessna_172 light plane.

spheres.py- contains 'vehicle' data for a bowling ball, used mainly for sanity-checks during bug fixing.
 
main_program- the code that runs the entire simulation; importing all the files above, it initialises the simulation, before using flat_earth_eom.py to return an array that is used for the visualisations. 


## 3: Physics

## 4: Getting started (Mac)

### 4.1: Requirements 

-Python 3.10+
-numpy
-math
-ussa1976 (USSA1976 atmospheric model)
-matplotlib

```
pip install numpy matplotlib ussa1976
```

### 4.2: Cloning the repository

```
git clone https://github.com/ad4mh07/6-DoF-Flight-Sim.git
cd 6-DoF-Flight-Sim
```

### 4.3: Running & customising the simulation for yourself
To run it, simply run main_program.py. 

There is a second pre-made scenario below "Vehicle & flight definition", greyed-out as a comment.

For more customisation, choose, import and alter the code for any combination of vehicle and initial conditions.

For even more flexibility, you can write your own vehicle data, and initial conditions. These conditions can be written in two formats, the only thing you will have to change is which function is called to build the initial array, x0.


## 5: Examples

## 6: Author and References
Written by Adam Hancock — Mathematics undergraduate, University of Bath. Built as a self-study project applying real analysis, linear algebra, and numerical methods to rigid-body flight dynamics. 

Disclaimer: AI (Claude) was used for consultancy and bug fixing. 

References: Ben Dickinson, Learn Guidance and Control (YouTube / Patreon tutorial series)
 — the primary reference this project follows Standard 6-DoF flat-earth formulation as used in classical flight dynamics texts (e.g. Stevens & Lewis, Aircraft Control and Simulation)

## 7: Limitations & improvements 
There are some limitations / improvements to be made. Namely, the integrator could be improved to one with an adaptive step size, like RK44. Some minor elements of the physics have not been included, specifically external moments, aerodynamic forces and a lift/drag/side-force aerodynamic model (stability derivatives). For the first two, these can easily be added in, as they are currently just initialised as 0.


The simulation could also benefit from including quaternions and Euler-angle gimbal lock. Allowing the user to start the simulation at a septic time, t0 is redundant, as the initial conditions get applied regardless.
