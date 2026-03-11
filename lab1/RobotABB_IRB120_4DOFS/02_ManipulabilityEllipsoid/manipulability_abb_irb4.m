function [HT_W,x4_W, x4p_w, w4p_w, w4_w,Heax_W,S,U,w,EAxis_W] = manipulability_abb_irb4(Q,Qp,t,dt)
%MANIPULABILITY_ABB_IRB4 Calculates the manipulability index and the EF
%velocities (linear and angular)
%   This function will calculate the manipulability index and the principal
%   axis of the manipulability ellipsoid. This function also calculates the
%   linear and angular veloctities of the EF. These velocities are obtained
%   using the Jacobian and the time derivative of the Rotation. The
%   velocities will be compared in a plot.
% Input:
% Q: Joint position vector 4x1
% Qp: Joint velocity vector 4x1
% t: current time
% dt: Delta time (needed for derivative)
%
% Return:
% HT_W: Cell array of HTs wrt wcf (6 elements 4x4)
% x4_W: position ef wrt wcf 3x1
% w4p_w: homogeneaous angular velocity wrt wcf (from Jacobian) 4x1
% w4_w: homogeneaous angular velocity wrt wcf (numerically) 4x1
% Heax_W: HT ellipsoid wrt to wcf
% S,U: SVD Jacobian linear velocities
% w: manipulability index (1x1)
% EAxis_W: ellipsoid principal axis (3x3)

persistent R4_0_old

%% Kinematics

% Kinematic Parameters L1,L2,L7,L8 and alpha
%TODO: finish the function abbIRB4_params
L=abbIRB4_params;

%% HT robot base (link0) wrt world cf. In this case is I (4x4)
%TODO: Define the HT of the robot base (link 0) w.r.t. wcf (w)
H0_W=eye(4);
%TODO: Extract the Rotation of the robot base wrt wcf
R0_W=H0_W(1:3,1:3);

%% Get the absolute HTs for all the joints with the new joint position
%TODO: Finish the function getAbsoluteHT_abbIRB4
[HT_0, HT_W] = getAbsoluteHT_abbIRB4(Q,L, H0_W);

%% Homogeneous transformation of Link4 wrt base frame (link0)
%TODO: Get the HT of the end-effector (ef) wrt base link (0) and wcf (w)
H4_0=HT_0{5};
H4_W=HT_W{6};

% Orientation of Link4 wrt base frame
%TODO: get the orientation of the ef wrt link 0
R4_0=H4_0(1:3,1:3);

%% Compute Position of Link4 with respect to the wcf
%TODO: Get the position of the ef wrt wcf 
x4_W=H4_W(1:3,4);

%% Compute the Linear and Angular velocities of Link4 (6X1)
%TODO: finish the function DifFK_abbIRB4
Xp4_0=DifFK_abbIRB4(Q, Qp, L);

% Linear Velocity (Link4) wrt base frame (homogeneous form) (4x1)
% This velocity is obtained from the differential kinematics (Jacobian)
%TODO: get the linear velocity of the ef
x4p_0=[Xp4_0(1:3);0];
% homogeneous Linear Velocity (Link4) wrt world coordinate frame (4x1)
%TODO: map the linear velocity to the wcf (w)
x4p_w = [R0_W * x4p_0(1:3); 0];

% Angular Velocity (Link4) wrt base frame (homogeneous 4x1)
% This velocity is obtained from the differential kinematics (Jacobian)
%TODO: get the angular velocity of the ef wrt link 0
w4_0=[Xp4_0(4:6)];
% Angular Velocity (Link4) wrt world coordinate frame (homogeneous 4x1)
%TODO: map the angular velocity to the wcf (w)
w4p_w=[R0_W * Xp4_0(4:6); 0];

%% Manipulability Analisys (Ellipsoid)
%TODO: finish the function J_EF_abbIRB4
jef_0=J_EF_abbIRB4(Q, L);

% Only the sub-jacobian for linear velocity
%TODO: Get the jacobian for the ef linear velocities
jef_v=jef_0(1:3,:);
disp(jef_v)
disp("*********")
pause(1.5)   % 暂停 1.5 秒

% Reduced form of U with dimension of numel(s). Used to compute the axis of
%the Manipulability ellipsoid axis_i=u_i*s_i
[U,S,V] = svd(jef_v,0);

%U is a rotation matrix (verification)
dU=det(U);
nU=norm(U);
IU=U*U';

su=size(U);
ss=size(S);

ru=rank(U,0.01);

% Manipulability Index
%TODO: compute the manipulability index
w = sqrt(det(jef_v * jef_v.'));

% Ellipsoid: Principal Axes
% Rotation includes the robot base to world orientation and the orientation of
% the principal axes of the m-ellipsoid
EAxis_W=R0_W*U;

%Scaling (homogeneous version of S)
S(4,4)=1;

% The total homogeneous transformation for the manipulability ellipsoid 
% includes the orientation wrt world and the ef position wrt 
% world as the center of the m-ellipsoid

Heax_W=[EAxis_W x4_W; 0 0 0 1];

%% Derivative of a Rotation Matrix (M.W. Spong, S. Hutchinson, and
% M. Vidyasagar. Robot modeling and control. pp 117)

if (t==0.0)
    R4_0_old=R4_0;
end

% Time derivative of R4_0
%TODO: Calculate the time derivative of R4_0 (use dt)
R4_0p=(R4_0-R4_0_old)/dt;

R4_0_old=R4_0;

% Skewsymetric matrix (S([wx;wy;wz])=dR/dt*R')
%TODO: Calculate the S matrix to get the ef's angular velocities wrt link 0
Sw_0=R4_0p*R4_0';

% Angular velocity vector of the ef wrt base frame
% This angular velocity is calculated using S
%TODO: Get the angular velocities from S
w4_0=[Sw_0(3,2);-Sw_0(3,1);Sw_0(2,1)];
% Angular velocity vector of the EF wrt wcf (homogeneous 4x1)
%TODO: Map the angular velocities to the wcf
w4_w=[R0_W*w4_0;0];

end