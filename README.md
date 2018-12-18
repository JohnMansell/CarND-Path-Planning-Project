# CarND-Path-Planning-Project
Self-Driving Car Engineer Nanodegree Program


>### Goals
> The goal of this project is to safely navigate around a virtual highway. The car must :
> * Avoid collisions
> * Observe the speed-limit
> * Stay in its lane, unless changing lanes
> * Plan an optimized path for traveling around the highway.
>
>### Data
> In this project, the simulator took care of localization and sensor fusion data. That is, the simulator provided a json string which
included the localization data for my car, as well as the relative locations of each of the other cars on the map on my side of the highway.

#
# Update

>### Position and Other Cars
> The first step in the path planning cycle is to update the current state. This includes the position, velocity, lane, etc of my vehicle
and the other vehicles on the road. This information is extracted from the json object returned by the simulator. Some of the data was
reformatted into more useful data, or extrapolated from the data given. For example, given the distance from the center line, I was able to
extract the lane position of each of the other cars on the road, even though this information was not given directly.

#
# Path Planning

>### Don't Crash
> The first step in the path planning process was to ensure I didn't crash into the car in front of me. To do this, I checked the distance
between my car, and the car directly in front of me at each step. If the distance was less than the safety buffer (3m) I applied a negative
acceleration to my vehicle.
>
>### Finite State Machine
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
>
>### Calculate Cost
> The car was designed to move to the lane with the lowest cost. The cost of each lane was determined by whether or not I could drive
freely in that lane. I also used the cost function to ban any lane changes which would take the car off the road. Then, based on the
lowest relative cost, I set the current and next state attributes of the finite state machine.
>
>### Execute Lane Change
> Once I had determined the next state, there were a series of functions which
would check if the particular lane change was safe. If it was safe, I would make the lane change. If it wasn't safe, I would wait until it
became safe, or sometimes alter the behavior of the vehicle to try and get my car into a position where it was safe to make the lane change.
For example, if my goal was to move over two lanes, but the lane next to me was blocked, I would drop back or speed up so that the lane
next to me would be clear. Then I could safely execute the two lane changes necessary to get to my target lane.
>
>### Complete Lane Change
> Once the lane change was executed, I would reset the finite state machine to the "keep lane" state. This effectively freed the finite
state machine to once again begin searching for the most optimum next state, and the cycle would continue. I would also adjust my speed
to be appropriate for my new lane. The updated speed was set to be the fastest I could drive while not crashing into any other vehicles,
and while observing the speed limit of the highway.
>
#
# Driving

>### Trajectory Generation
> Once the optimum path had been calculated, the next step was to generate the appropriate trajectory for the vehicle. The first few points
of the trajectory were extracted from the last path of the vehicle. This helped to smooth out transitions, and saved the computer from having
to calculate all 50 trajectory points each time. Only the remaining trajectory points needed to be calculated. If no previous path points
were available, I extrapolated a path back behind the current position of the vehicle. This helped to ensure that the trajectory of the
vehicle was as smooth as possible.
>
>### Map Way Points
> Given the current state of the vehicle, the previous path of the vehicle, and the target lane, speed, and distance, waypoints were set on
map which most closely correlated with the desired trajectory of the vehicle.
>
>### Spline
> Once the waypoints were set, I used the spline.h header file to calculate a spline which would necessarily pass through each of those
waypoints. I also manually forced the spline to have a 0 second derivative at each end of the trajectory. This helped to smooth out
transitions and minimize the jerk and acceleration.
>
>### Next X Vals, Next Y Vals
> Once the spline had been calculated, points along that spline could be easily calculated. These are the points in (x, y) coordinates
which the car should drive in order to execute the desired path. These points were then passed along to the simulator, which handled
steering the car through each of the (x, y) points.

#
# Future Improvements
>
> ### More Optimize trajectories
> Currently, there are a few situations the vehicle doesn't handle well. In particular, if it would be best to move two lanes to the right
or to the left, it can sometimes get stuck in a "pocket" with one car in front, and one car to the side. The "Wait or drop back" function
would need to be further tweaked to help it make these decisions better.
>
> ### Spline Cost functions
> Another feature I would like to include is a cost function for multiple splines. Currently, the whole process only calculates a single
spline. A better approach would be to calculate multiple splines, and then have cost functions associated with speed, acceleration, jerk,
final position, safety, etc. This would allow for a more optimized path around the track. The current splines have restrictions placed on
them which aren't necessary in every situation, but have to be observed because in some situations the generated spline would create
too much jerk or acceleration otherwise.
