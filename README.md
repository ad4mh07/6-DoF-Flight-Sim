# 6 Degree of Freedom Flight Simulator

## 1: Overview
I have built a 6 degree of freedom flight simulator in Python. Taking a vehicle model and initial flight conditions, it plots the flight path, and other important visualisations, such as angle of attack and side slip, velocities, etc. The core of the model, flat_earth_eom.py 

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

## 4: Running the simulation for yourself

## 5: Examples

## 6: Author

## 7: Limitations & improvements 
integartor
external moments, etc
