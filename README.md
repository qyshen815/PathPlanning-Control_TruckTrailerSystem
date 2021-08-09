# PathPlanning_TruckTrailerSystem
Project for the lecture Automotive Control Systems building an autonomous truck trailer system including Path Planning, Trajectory Generation and Control


https://user-images.githubusercontent.com/68558072/127210936-aed3ce1e-849f-4d84-bc43-37d42c7ef222.mp4

The goal of the project is to create a Matlab Framework where a vehicle travels together with a trailer, for instance a Truck/Trailer system, from a random initial pose to an arbitrary final pose, given a specified path. This must perform both forward and backwards motion. The desired final pose shall be reached, even if the actual initial pose differs from the one presumed for the path planning (Wirtensohn, Lecture 1). Therefore, a controller needs to be designed that ensures that the center point of the Trailer axel follows a predefined trajectory, described by a geometric path and a velocity profile. The derivatives of the path at the start and end point are defined by the Truck/Trailer configuration and the steering angle of the Truck (Wirtensohn). 
