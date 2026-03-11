function Xp=DifFK_abbIRB4(q,qp,L)
%DIFFK_ABBIRB4 calculates the end-effector's linear and angular velocity
%vector wrt the robot base (link 0)
% input:
%   q: is the joint position vector 1X4
%   qp: is the joint velocity vector 1X4
%   L: is the kinematic parameter array (see abbIRB4_params.m)
% return:
%   Xp: End-effector linear and angular velocitites wrt robot base (6X1)


% Joint positions
q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

% Joint velocities
qp1=qp(1);
qp2=qp(2);
qp3=qp(3);
qp4=qp(4);

% Kinematic paramters
L1=L(1);
L2=L(2);
L7=L(3);
L8=L(4);
al=L(5);

% Common substitutions
%c1=cos(q1);
%s1=sin(q1);

%c2=cos(q2);
%s2=sin(q2);

%c234=cos(q2 + q3 + q4);
%s234=sin(q2 + q3 + q4);

%ca23=cos(al + q2 + q3);
%sa23=sin(al + q2 + q3);


% Define the absolute HT wrt robot base (link 0)
theta = [q1; q2 - pi/2; q3 + al; q4 - al + pi/2];
d = [L1; 0; 0; 0];
alpha = [-pi/2; 0; 0; 0];
a = [0; L2; L7; L8];

T = cell(4,1);

length = size(theta,1);
for i = 1:length
    T{i} = [cos(theta(i)), -sin(theta(i)) * cos(alpha(i)),  sin(theta(i)) * sin(alpha(i)), a(i) * cos(theta(i));
            sin(theta(i)),  cos(theta(i)) * cos(alpha(i)), -cos(theta(i)) * sin(alpha(i)), a(i) * sin(theta(i));
            0, sin(alpha(i)), cos(alpha(i)), d(i);
            0, 0, 0, 1];
end

T1_0 = T{1};

T2_0 = T{1} * T{2};

T3_0 = T{1} * T{2} * T{3};

T4_0 = T{1} * T{2} * T{3} * T{4};

% TODO: define the Jacobian of the ef wrt robot base (link 0) 6xn
Z = zeros(3, 5);
T = zeros(3, 5);

Z(:, 2:5) = [T1_0(1:3,3), T2_0(1:3,3), T3_0(1:3,3), T4_0(1:3,3)];
Z(:, 1) = [0; 0; 1];
T(:, 2:5) = [T1_0(1:3,4), T2_0(1:3,4), T3_0(1:3,4), T4_0(1:3,4)];
Jef = zeros(6,4);
for i= 1:4
    Jef(:, i) = [cross(Z(:,i), (T(:,5) - T(:,i))); Z(:,i)];
end

%TODO: Calcualte the linear and angular velocity vector of the ef wrt 0
Xp=Jef * [qp1; qp2; qp3; qp4];
end