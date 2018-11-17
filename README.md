# COMSM4111 Robotic Systems - Individual Coursework

## Simulated Robot Localisation
We were provided with a MATLAB simulation of a robot which has ultrasound sensors to measure distance to objects. The robot is placed at a random position and orientation within a map. We were tasked with developing a localisation system that takes the robot from its start position to a specified target position.

## Solution
The robot uses a particle filter to work out its position and orientation within the map. The map is dicretized so that the A* search algorithm can be used to plan the robot's path towards the target position. The particle filter is used continuously as the robot moves towards the target so that it remains certain of its position. If the robot strays away from its intended path then the path is recalculated.

## Running the Simulation
Run the test.m file in MATLAB. The circle with the blue line represents the robot's actual position and orientation. The circle with the red line represents the robot's predicted position and orientation, calculated using the particle filter. The X marks the target position.

## Notes
- The robot doesn't always end up near the target position. I think this is due to a bug in the A* search implementation.
