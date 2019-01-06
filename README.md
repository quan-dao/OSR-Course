# OSR-Course

[//]: # (Image References)

[image1]: ./path_planning_RSM.png
[image2]: ./path_planning_RRT.png
[image3]: ./path_planning_RRT_timestamp.png

## Introduction
This repo stores my solutions to the exercises in Chapter 2, 3 & the Motion Planning assignment in the course http://www.osrobotics.org/osr/. 

## Chapter 2
Solutions to the forward & inverse kinematics exercise are respectively implimented in **tut_fk.py** and **tut_ik.py**

## Chapter 3
The Probabilistic Road Map (PRM) is implemented in **tut_path_planning.py**. The path found by PRM is displayed in Fig.1

![alt text][image1]

*Fig.1 The path found by PRM (green circles are nodes sampled by PRM)*

The Rapidly explored Random Tree (RRT) is implemented in **tut_path_planning_RRT.py**. This file also containts the implementation of the time-parameterization of the generated path. The path found, the velocity and the acceleration in x, y direction are respectedly displayed in Fig.2 and Fig.3.

![alt text][image2]

*Fig.2 The path found by bidirectional RRT (green circles are nodes of each tree)*

![alt text][image3]

*Fig.3 The velocity and acceleration along the path found by RRT (maximum velocity and accerleration are 0.2 and 0.05)*

The RRT is much faster than PRM. RRT takes 4.9 second to find a path, while the PRM needs 10.2 second.

## Motion Planning assignment
The solution to this assignment is implemented in **pick_and_place.py**. Its description is in the *assignment_report.pdf*.
