# Path Planning
> [![Path Planning](WriteUp/PathPlanning.gif)](https://youtu.be/hQwN9OZ39WY "Path Planning")  
> [Full Video](https://youtu.be/hQwN9OZ39WY) on YouTube

# Objective
> The goal of this project is to build a path planner that creates
smooth, safe trajectories for the car to follow. The highway track has
other vehicles, all going different speeds, but approximately obeying
the 50 MPH speed limit.
>
> The car transmits its location, along with its sensor fusion data, which estimates the location of all the vehicles on the same side of the road.

# What I Learned
> - A* search algorithm
> - Dynamic programming path planning
> - Environment prediction
> - Behavior planning
> - Finite state machine
> - Trajectory generation
>
> ### Languages
> C++

# Video

### Included Files
> [WriteUp.md](WriteUp/WriteUp.md) -- Full Write Up  
> [Vehicle.cpp](src/Vehicle.cpp) -- Vehicle object, self and other cars on the road  
> [helper_functions.cpp](src/helper_functions.cpp) -- General functions for processing data, mostly math and waypoint analysis.  
> [main.cpp](src/main.cpp) -- Connects to the simuator. Process data and send commands back to the car through the simulator.  

