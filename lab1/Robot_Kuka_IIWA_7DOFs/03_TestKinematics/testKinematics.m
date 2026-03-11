% Adds the additional folders with functions into the library path
addpath('../DrawCFs')
addpath('../Model')

% Defines the robot variables needed for the simulation
robot_name="kukaIiwa7"; 

% Load the robot
robot = loadrobot(robot_name,"DataFormat","column","Gravity",[0 0 -9.81]);

% Test joint trajectory (sinusoidal)
q=(-pi:0.2:pi)';

% Vector of zeros
zq=zeros(numel(q),1);

% Generate the different joint positions (states) to assess your kinematic
% model
stateData=[   0,   0,   0,    0,   0,   0,    0;
           pi/2,   0,   0,    0,   0,   0,    0;
              0,pi/2,   0,    0,   0,   0,    0;
              0,   0,pi/2,    0,   0,   0,    0;
              0,   0,   0, pi/2,   0,   0,    0;
              0,   0,   0,    0, pi/2,  0,    0;
              0,   0,   0,    0,   0, pi/2,   0;
              0,   0,   0,    0,   0,   0, pi/2;
          -pi/2,   0,   0,    0,   0,   0,    0;
              0,-pi/2,  0,    0,   0,   0,    0;
              0,   0,-pi/2,   0,   0,   0,    0;
              0,   0,   0, -pi/2,   0,   0,    0;
              0,   0,   0,    0,-pi/2,  0,    0;
              0,   0,   0,    0,   0, -pi/2,   0;
              0,   0,   0,    0,   0,   0, -pi/2;
              q,   q,   q,    q,   q,   q,    q];

close all

% Visualize robot: First start the kinematic model verification with the 
% robot visualization ('on'). Then you can fine tune the evaluation only
% with the coordinate frames ('off').
vr='on';%'off';

plotRobot_tk(robot,stateData,vr);



