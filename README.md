# RobotNavigation_BehavioralControl_2014
Arduino code and project documentation for behavioral control design for robot navigation in cluttered indoor environments

The project involves navigation of a wheeled robot in an unknown
cluttered environment. The differentially driven mobile robot is modeled as a unicycle for control design. Wheel odometry (using Infrared sensors)
is used to track the position and orientation (pose) of the robot and SONAR is used to
sense obstacles in the environment. A behavior based approach with
three controllers (behaviors), namely, go-to-goal, avoid obstacle and
follow wall is employed. A supervisor that switches between these
controllers using appropriate guards and reset conditions is designed
for robust navigation. A PID regulator ensures smooth movement of the robot towards the goal while avoiding obstacles in the environment.
