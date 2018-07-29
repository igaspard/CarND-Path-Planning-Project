# CarND Path Planning Project - Gaspard Shen

The first project of the Term 3 Udacity Self Driving Car Nanodegree. The goal of this project is to design a path planner able to create smooth and safety first path for the car running on the 3 land highway with other cars. The result was great! The maximum distance of the car can go was 12.48 miles without any incident.

![](/Results/Screen Shot.png)

## Implementations
![Path Planning Block Diagram](/Results/BlockDiagram.png)

Above picture is the major idea and components of the path planning. By using the technique we learn from the lecture such as Finite State Machines for behavior planner whether we should change lane or keep in the lane by the different cost functions. Then an idea from the Q&A video, build the new waypoints from the previously formed path.

### FSM
There are 6 states of our FSM to control how our car behavior. Here explain how our FSM works.
1. Constant speed(**CS**): Initial state.
2. Keep Line(**KL**): According to our logic, we will increase the car's speed if no cars ahead or decrease when there are cars in front of us. And no exceed the  speed limit.
3. Prepare Line Change Left(**PLCL**): Once we found there is a car in front of us and we can make it fastest, we will move to this state.
4. Prepare Line Change Right(**PLCR**): Same as PLCL.
5. Lane Change Left(**LCL**): Once we are in the PLCL state, we need to consider whether there are cars in front of the left lane or in the back it in our safe distance.
6. Lane Change Right(**LCR**): Same as LCL.

### Cost functions
Here we have 5 cost functions in our FSM design.
1. Lane Change: The weight how we keep stay in the same lane.
2. Efficiency: How aggressive we change the lane if another can driver faster.
3. Free Lane: Similar as above one, will move the car to free lane that no car ahead.
4. Collision: If collision was found in other lane, we don't want to change it. This one has the highest weight.
5. Buffer: This weight decide how we move between the PLCL/PLCR to LCL/LCR.

## Results
In the end, my implementation can perform the maximum 12.48 miles without any incident. Below is the YouTube Link recording part of how the car running for reference.

[YouTube Link of My Best Record](https://youtu.be/Zv4EOextHTQ)
