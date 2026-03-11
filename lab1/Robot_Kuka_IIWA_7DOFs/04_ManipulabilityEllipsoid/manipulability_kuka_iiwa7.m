function [HT_W,x7_W, x7p_w, w7p_w, w7_w,Heax_W,S,U,w,EAxis_W] = manipulability_kuka_iiwa7(Q,Qp,t,dt)
%MANIPULABILITY_KUKA_IIWA7 Calculates the manipulability index and the EF
%velocities (linear and angular) 
%   This function will calculate the manipulability index and the principal
%   axis of the manipulability ellipsoid. This function also calculates the
%   linear and angular veloctities of the EF. These velocities are obtained
%   using the Jacobian and the time derivative of the Rotation. The
%   velocities will be compared in a plot.
% Input:
% Q: Joint position vector 7x1
% QP: Joint velocity vector 7x1
% t: current time
% dt: Delta time (needed for derivative)
%
% Return:
% HT_W: Cell array of HTs wrt wcf (9 elements 4x4)
% x4_W: position ef wrt wcf 3x1
% w4p_w: homogeneaous angular velocity wrt wcf (from Jacobian) 4x1
% w4_w: homogeneaous angular velocity wrt wcf (numerically) 4x1
% Heax_W: Ht ellipsoid wrt to wcf
% S,U: SVD Jacobian linear velocities
% w: manipulability index (1x1)
% EAxis_W: ellipsoid principal axis (3x3)

persistent R7_0_old

%% Kinematics

% TODO: Define the kinematic Parameters of your robot
% Kinematic Parameters L1,L2,L3, and L4
L=kukaIIWA7_params;


%% HT robot base (link0) wrt world cf. In this case is I (4x4)
%TODO: Define the HT of the robot base (link 0) w.r.t. wcf (w)
H0_W=eye(4);
%TODO: Extract the Rotation of the robot base wrt wcf
R0_W=H0_W(1:3,1:3);

%% Get the absolute HTs for all the joints with the new joint position
%TODO: Finish the function getAbsoluteHT_kukaIIWA7
[HT_0, HT_W] = getAbsoluteHT_kukaIIWA7(Q,L, H0_W);

%% Homogeneous transformation of Link7 wrt base frame (link0)
%TODO: Get the HT of the end-effector (ef) wrt base link (0) and wcf (w)
H7_0=HT_0{8};
H7_W=HT_W{9};


% Orientation of Link7 wrt base frame
%TODO: get the orientation of the ef wrt link 0
R7_0=H7_0(1:3,1:3);

%% Compute Position of Link7 with respect to the wcf
%TODO: Get the position of the ef wrt wcf (3x1)
x7_W=H7_W(1:3,4);

%% Compute the Linear and Angular velocities of Link7 (6X1)
%TODO: Finish the function DifFK_kukaIIWA7
Xp7_0=DifFK_kukaIIWA7(Q, Qp, L);

% Linear Velocity (Link7) wrt base frame (homogeneous form) (4x1)
% This velocity is obtained from the differential kinematics (Jacobian)
%TODO: get the linear velocity of the ef
x7p_0=[Xp7_0(1:3);0];
% Linear Velocity (Link7) wrt world coordinate frame (4x1)
%TODO: map the linear velocity to the wcf (w)
x7p_w=[R0_W * Xp7_0(1:3);0];

% Angular Velocity (Link7) wrt base frame
% This velocity is obtained from the differential kinematics (Jacobian)
%TODO: get the angular velocity of the ef wrt link 0 (homogeneous 4x1)
w7_0=[Xp7_0(4:6);0];

% Angular Velocity (Link7) wrt world coordinate frame
%TODO: map the angular velocity to the wcf (w) (homogeneaous 4x1)
w7p_w=[R0_W * Xp7_0(4:6);0];

%% Manipulability Analisys (Ellipsoid)
%TODO: finish the function J_EF_kukaIIWA7
jef_0=J_EF_kukaIIWA7(Q, L);

% Only the sub-jacobian for linear velocity
%TODO: Get the jacobian for the ef linear velocities
jef_v=jef_0(1:3,1:7);

%TODO: Get the jacobian for the ef angular velocities
jef_w=jef_0(4:6,1:7);

% Reduced form of U with dimension of numel(s). Used to compute the axis of
%the Manipulability ellipsoid axis_i=u_i*s_i
[U,S,V] = svd(jef_w,0);

%U is a rotation matrix (verification)
dU=det(U);
nU=norm(U);
IU=U*U';

su=size(U);
ss=size(S);

ru=rank(U,0.01);

% Manipulability Index
%TODO: compute the manipulability index
w= sqrt(det(jef_v * jef_v.'));

% Ellipsoid: Principal Axes
% Rotation includes the robot base to world orientation and the orientation of
% the principal axes of the m-ellipsoid
EAxis_W=R0_W*U;

%Scaling (homogeneous version of S)
% TODO: Get the principal axis for the linear velocities
S = S(1:3,1:3);
S(4,4)=1;

% The total homogeneous transformation for the manipulability ellipsoid 
% includes the orientation wrt world and the ef position wrt 
% world as the center of the m-ellipsoid

Heax_W=[EAxis_W x7_W; 0 0 0 1];

%% Derivative of a Rotation Matrix (M.W. Spong, S. Hutchinson, and
% M. Vidyasagar. Robot modeling and control. pp 117)

% Time derivative of R7_0 
if (t==0.0)
    R7_0_old=R7_0;
end

%TODO: Calculate the derivative of R7_0 (use dt)
R7_0p=(R7_0-R7_0_old)/dt;

R7_0_old=R7_0;

% Skewsymetric matrix (S([wx;wy;wz])=dR/dt*R')
%TODO: Calculate the S matrix to get the ef's angular velocities wrt link 0
Sw_0=R7_0p*R7_0';

% Angular velocity vector of the ef wrt base frame
% This angular velocity is calculated using S
%TODO: Get the angular velocities from S
w7_0=[Sw_0(3,2);-Sw_0(3,1);Sw_0(2,1)];
% Angular velocity vector of the EF wrt wcf (homogeneous 4x1)
%TODO: Map the angular velocity wrt wcf
w7_w=[R0_W*w7_0;0];


end

