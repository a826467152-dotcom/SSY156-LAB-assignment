# Task 2 Manipulability Analysis and Jacobian evaluation Kuka Robot

## Description

In this example, we will test the absolute homogeneous transformations (HT)
and the Jacobians obtained from the DH table. The robot will be simulated using predefined
joint positions and a trajectory. The assessment is in two ways:

1. The simulator will compare the linear and angular velocities of the end-effector (ef) obtained from the Differential kinematics (Jacobians) with the linear and angular velocities of the end-effector calculated numerically, i.e., time derivative of position and orientation of the end-effector wrt robot base (link 0).
2. The simulator will plot the manipulability ellipsoid using the linear velocity Jacobian of the end-effector wrt base frame.

This tasks will use the results obtained from the Task 3 "Test Kinematics". Therefore, verify your HT before solving this tasks.

### Files

```
.
└── Robot_Kuka_IIWA_7DOFs
    ├── 04_ManipulabilityEllipsoid
    │   ├── KSimulator_kuka_iiwa7.slx (Simulink model with the simulator) [you can change the type of joint command]
    │   ├── manipulability_kuka_iiwa7.m (function to compute the manipulability index, ellipsoid, and the linear and angular velocities) [follow the TODOs]
    │   ├── plotRobot.m (plotting function to visualize the robot) [nothing todo here!]
    │   └── run_kuka_iiwa7.m (main script to run the model and set global parameters) [nothing todo here!]
    ├── DrawCFs: Plotting functions [nothing todo here!]
    └── Model
        ├── kuka_iiwa7_params.m (function with the Kinematic parameters) [follow the TODOs]
        ├── DifFK_kuka_iiwa7.m (calculates the linear and angular velocity vector of the end-effector) [follow the TODOs]
        ├── getAbsoluteHT_kuka_iiwa7.m (function to compute absolute HT wrt link 0 and wcf) [follow the TODOs]
        └── J_EF_kuka_iiwa7.m (calculates the jacobian of the end-effector wrt link 0 6Xn) [follow the TODOs]
```

## How2run

After finishing all the tasks marked as **TODO** in the files: **../Model/kuka_iiwa7_params.m**, **Model/getAbsoluteHT_kuka_iiwa7.m**, **plotRobot.m**, and **manipulability_abb_irb4.m** run the following commands:

1. use the folder 04_ManipulabilityEllipsoid as the workspace. In the matlab terminal
   run:

```bash
>> cd /path_to/Robot_Kuka_IIWA_7DOFs/04_ManipulabilityEllipsoid
```

2. Run the main script:

```bash
>> run_kuka_iiwa7
```

3. This will open the simulink model **KSimulator_kuka_iiwa7.slx**. Usually, the first time takes some time to load the simulink environment. To run the simulation, just click on "Run" or use the keyboard command "CTRL+T". When the model is running, you should see a robot and an ellipsoid in the end-effector.
   In the model, you can select between static and time-varying joint positions. The static positions are defined by the constant block "qd". Test your Jacobian in different joint configurations. The model has four scopes to visualize the comparative analysis of the linear and angular velocities. If your Jacobian is correct, then you should see only three lines in each plot, and the error shown in the other plots should be small (c.a. 10^-3).
