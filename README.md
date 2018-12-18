# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program


### Goals
> The goal of this project is to safely navigate around a virtual highway. The car must :
> * Avoid collisions
> * Observe the speed-limit
> * Stay in its lane, unless changing lanes
> * Plan an optimized path for traveling around the highway.

### Data
> In this project, the simulator took care of localization and sensor fusion data. That is, the simulator provided a json string which
included the localization data for my car, as well as the relative locations of each of the other cars on the map on my side of the highway.

#
# Update

### Position and Other Cars
> The first step in the path planning cycle is to update the current state. This includes the position, velocity, lane, etc of my vehicle
and the other vehicles on the road. This information is extracted from the json object returned by the simulator. Some of the data was
reformatted into more useful data, or extrapolated from the data given. For example, given the distance from the center line, I was able to
extract the lane position of each of the other cars on the road, even though this information was not given directly.

#
# Path Planning

### Don't Crash
> The first step in the path planning process was to ensure I didn't crash into the car in front of me. To do this, I checked the distance
between my car, and the car directly in front of me at each step. If the distance was less than the safety buffer (3m) I applied a negative
acceleration to my vehicle.

### Finite State Machine
> The decision making process was handled by a finite state machine model.
>
> The Available steps are:
> * Keep Lane
> * Change Left
> * Change Right
> * Prep 2 Left
> * Prep 2 Right
> * Prep Left
> * Prep Right
>
> If the vehicle was currently in the middle of a lane change,  the finite state machine would wait until that lane change was complete.
Otherwise, it would try to find the next best state and take the steps necessary to get there. The next best state was determined
by minimizing a cost function. The cost function was designed to penalize lanes in which the cars in front of me were driving slowly.
The goal of finding the next best state was to always be in the lane which would allow me to drive around the track most quickly.

### Calculate Cost
> The car was designed to move to the lane with the lowest cost. The cost of each lane was determined by whether or not I could drive
freely in that lane. I also used the cost function to ban any lane changes which would take the car off the road. Then, based on the
lowest relative cost, I set the current and next state attributes of the finite state machine. Then, there were a series of functions which
would check if the particular lane change was safe. If it was safe, I would make the lane change. If it wasn't safe, I would wait until it
became safe, or sometimes alter the behavior of the vehicle to try and get my car into a position where it was safe to make the lane change.
For example, if my goal was to move over two lanes, but the lane next to me was blocked, I would drop back so that I could
