## Main Structure

This Path Planning Project is constructed with three main module.

* Prediction
* Behavior Planning
* Trajectory Generation

## Prediction Module

This module takes **Sensor Fusion** data as input, and output **three bool variables** ,which are `car_ahead, car_left, car_right` .

Each of these three variables indicate whether there are cars aheads, on the right lane or on the left lane.

1. To get the each car's `d, vx, vy, s` by iterating `sensor_fusion` vector, and calculate each car's speed by `sqrt(vx * vx + vy * vy)`
2. Check the `d` to know which lane each car is in. If d is negative, it means the car is on the other half of the road going the other direction, to avoid noise, I assign ` the check_car_lane` to 99
3. If the other car's lane equals ego car's lane and the distance to ego car is less then 30, and the speed is smaller than ego car, then mark `car_ahead` to true
4. By checking the difference of the other car's lane and ego car's, we can understand is it on the right hand side or the left hand side. To let the module be more safe, I also take distance difference and speed difference in to consideration. If there are car on the left/right lane with distance from -30 to 50 and is slower then ego car, then assign ` car_left` or `car_right` as true, which means it maybe dangerous to change the lane.

## Behavior Planning Module

This module use couple of `if..` to check all kinds of situation.

If in front of ego car, there is a slow car within 30 meters, we might need to consider to change the lane. So we list out conditions that the ego car can change the lane:

* If ego car is on lane 0 (can not change to left), and there is no car within -50 to 50 range on the RIGHT lane, then change to RIGHT lane.

* If ego car is on lane 2 (can not change to right), and there is no car within -50 to 50 range on the LEFT lane, then change to LEFT lane.
* If ego car is on lane 1, and there is no car within -50 to 50 range on the LEFT lane, then change to LEFT lane.
* If ego car is on lane 2, and there is no car within -50 to 50 range on the RIGHT lane, then change to RIGHT lane.

All the other conditions might make changing lane unfeasible or dangerous. If changing lane is not possible, then ego car need to slow down.

## Trajectory Generation Module

I use SPLINE libarary to generate the trajectory as show in the Q&A video.

Finally the car was able to travel more than 8 mile without incident.

![1668934061429](image/MODELDOCUMENTATION/1668934061429.png)

## Shortage of This Model

1. Behavior Module is tend to be conservative (-50 to 50 range check is relatively big), so the ego car may tend to be stay in the current lane than switch to the other. This sacrifice the efficiency.
2. Did not use cost function when it comes to make the decision which behavior to take. So it may not be able to deal with more complicate situation other than just highway. Such as there is a car break down on the road, the brake acceleration or the trajectory might be to gentle to react more quickly.
