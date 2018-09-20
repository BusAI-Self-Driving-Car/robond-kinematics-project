## Project: Kinematics Pick & Place

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image2]: ./misc_images/misc3.png
[image3]: ./misc_images/misc2.png

[image-kr210]: ./writeup_images/kr210.png
[image-implement]: ./writeup_images/fk-vs-ik.png
[image-total-transform]: ./writeup_images/total-transform.png
[image-inididual-transform]: ./writeup_images/inididual-transform.png
[image-fk-dh]: ./writeup_images/fk-dh.png
[image-dh-table]: ./writeup_images/dh-table-handdraft.png
[image-ik-1]: ./writeup_images/ik-1.png
[image-ik-3]: ./writeup_images/ik-3.png
[image-inverse-position-1]: ./writeup_images/inverse-position-1.png
[image-inverse-position-2]: ./writeup_images/inverse-position-2.png
[image-inverse-rotation]: ./writeup_images/inverse-rotation.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

Here is an example of how to include an image in your writeup.

![alt text][image1]

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | L1 | qi
1->2 | - pi/2 | L2 | 0 | -pi/2 + q2
2->3 | 0 | 0 | 0 | 0
3->4 |  0 | 0 | 0 | 0
4->5 | 0 | 0 | 0 | 0
5->6 | 0 | 0 | 0 | 0
6->EE | 0 | 0 | 0 | 0


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

And here's where you can draw out and show your math for the derivation of your theta angles. 

![alt text][image2]

### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


Here I'll talk about the code, what techniques I used, what worked and why, where the implementation might fail and how I might improve it if I were going to pursue this project further.  


And just for fun, another example image:
![alt text][image3]



========================

## Introduction

This project is to implement a Pick and Place task with a KR-210 robotic arm. Forward Kinematics and Inverse Kinematics are applied to automatically generate a viable path or trajectory for the arm and gripper and grasp the target and put it into the specific area. 

Example image:

![alt text][image3]

A KR-210 robotic arm:

![alt text][image-kr210]

## Implementation

![alt text][image-implement]

### Forward Kinematics

In the FK problem, we know all the joint variables, that is the generalized coordinates associated with the revolute and prismatic joints, and we wish to calculate the pose of the end effector in a 3D world. 

#### Denavit-Hartenberg (DH) Parameters

To do FK and IK, we are using a method by Jacques Denavit and Richard Hartenberg which requires only four parameters for each reference frame.

![alt text][image-fk-dh]

Looking up the "kr210.urdf.xacro" file, the initial positions are as follows

joint_0 = (0, 0, 0)
joint_1 = (0, 0, 0.33)
joint_2 = (0.35, 0, 0.42)
joint_3 = (0, 0, 1.25)
joint_4 = (0.96, 0, -0.054)
joint_5 = (0.54, 0, 0)
joint_6 = (0.193, 0, 0)
joint_G = (0.11, 0, 0)

For better understanding, I also draw it manually as follows,

![alt text][image-dh-table]

Following are DH parameters obtained based on the above information and graph:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | -pi/2 | 0.35 | 0 | q2 - pi/2
2->3 | 0 | 1.25 | 0 | q3
3->4 |  -pi/2 | -0.054 | 1.5 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | -pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0

#### Homogenous transform

Homogenous transforms are then combined together. Parameters of each transformation are set from DH parameters.

The FK problem boils down to the composition of homogeneous transforms. We start with the base link and move link by link to the end effector. And we use the DH parameters to build each individual transform.

The total transform:

![alt text][image-total-transform]

The individual transform:

![alt text][image-individual-transform]

### Inverse Kinematics

Inverse Kinematics (IK) is the exact opposite of FK, where we calculate the parameters from a given coordinate position and rotation.

Since the last three joints in our robot are revolute and their joint axes intersect at a single point, we have a case of spherical wrist with joint_5 being the common intersection point and hence the wrist center.

This allows us to kinematically decouple the IK problem into Inverse Position and Inverse Orientation problems,

![alt text][image-ik-1]

#### Inverse position

First let us solve for the Inverse Position problem. Since we have the case of a spherical wrist involving joints 4,5,6, the position of the wrist center is governed by the first three joints. We can obtain the position of the wrist center by using the complete transformation matrix we derived in the last section based on the end-effector pose.

For the sake of simplification, let us symbolically define our homogeneous transform as following:

![alt text][image-inverse-position-1]

where l, m and n are orthonormal vectors representing the end-effector orientation along X, Y, Z axes of the local coordinate frame.

Since n is the vector along the z-axis of the gripper_link, we can say the following:

![alt text][image-inverse-position-2]

Where,

Px, Py, Pz = end-effector positions

Wx, Wy, Wz = wrist positions

d6 = from DH table

_l_ = end-effector length

Then we have

```Rrpy = Rot(Z, yaw) * Rot(Y, pitch) * Rot(X, roll) * R_corr```

Where R_corr is the correctional rotation matrix.

Now nx, ny, and nz values can be extracted from this Rrpy matrix to obtain the wrist center position.

#### Inverse rotation

![alt text][image-inverse-rotation]

The labels 2, 3 and WC are Joint 2, Joint 3, and the Wrist Center, respectively. You can obtain, or rather visualize, the triangle between the three if you project the joints onto the z-y plane corresponding to the world reference frame. From your DH parameters, you can calculate the distance between each joint above. Using trigonometry, specifically the Cosine Laws, you can calculate theta 2 and theta 3.

For the Inverse Orientation problem, we need to find values of the final three joint variables.

Using the individual DH transforms we can obtain the resultant transform and hence resultant rotation by:

```R0_6 = R0_1*R1_2*R2_3*R3_4*R4_5*R5_6```

Since the overall RPY (Roll Pitch Yaw) rotation between base_link and gripper_link must be equal to the product of individual rotations between respective links, following holds true:

```R0_6 = Rrpy```

where,

Rrpy = Homogeneous RPY rotation between base_link and gripper_link as calculated above.

We can substitute the values we calculated for joints 1 to 3 in their respective individual rotation matrices and pre-multiply both sides of the above equation by inv(R0_3) which leads to:

```R3_6 = inv(R0_3) * Rrpy```

The resultant matrix on the RHS (Right Hand Side of the equation) does not have any variables after substituting the joint angle values, and hence comparing LHS (Left Hand Side of the equation) with RHS will result in equations for joint 4, 5, and 6.

