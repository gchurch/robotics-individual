# COMSM4111 Robotic Systems - Individual Coursework

## Simulated Robot Localisation
We were provided with a MATLAB simulation of a robot which has 360&deg; ultrasound sensors to measure distance. The robot is placed at a random position and orientation within a map. We were tasked with developing a localisation system that takes the robot from its start position to a specified target position. This was the individual part of the coursework, we also had a group part in which we used an actual robot.

## Solution
The robot uses a particle filter to work out its position and orientation within the map. The map is dicretized so that the A* algorithm can be used to plan the robot's path towards the target position. The particle filter is used continuously as the robot moves towards the target so that it remains certain of its position. If the robot strays away from its intended path then the path is recalculated.


