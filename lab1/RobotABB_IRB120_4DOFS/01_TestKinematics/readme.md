# Task 1 Test Kinematics

## Description

In this example, we will test the absolute homogeneous transformations (HT)
obtained from the DH table. The robot will be simulated using predefined
joint positions and a trajectory. The assessment is to visually verify that
the coordinate frames obtained from the HT models correspond to the
robot model.

### Files

```
.
└── RobotABB_IRB120_4DOFS
    ├── 01_TestKinematics
    │   ├── plotRobot.m (function to plot the robot and the CFs) [follow the TODOs]
    │   ├── readme.md (This readme file)
    │   └── testKinematics.m (main scripts to run the test) [nothing todo here!]
    ├── DrawCFs: Plotting functions [nothing todo here!]
    └── Model
        ├── abbIRB4_params.m (function with the Kinematic parameters) [follow the TODOs]
        └── getAbsoluteHT_abbIRB4.m (function to compute absolute HT wrt link 0 and wcf) [follow the TODOs]
```

## How2run

After finishing all the tasks marked as **TODO** in the files: **../Model/abbIRB4_params.m**, **Model/getAbsoluteHT_abbIRB4.m**, and **plotRobot.m**, run the following commands:

1. use the folder 01_TestKinematics as the workspace. In the matlab terminal
   run:

```bash
>> cd /path_to/RobotABB_IRB120_4DOFS/01_TestKinematics
```

2. Run the main script:

```bash
>> testKinematics
```

3. This will open a plot where you will see the robot and the CFs for all the links and the end-effector (ef)

4. Each time you press a mouse button on the plot a new joint position will be commanded. If your Homogeneous Transformations are properly defined you should see the coordinate frames in the right pose with respect to the robot.
