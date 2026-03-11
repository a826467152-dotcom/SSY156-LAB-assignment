function Jef_0=J_EF_abbIRB4(q,L)
%J_EF_ABBIRB4 calculates the geometric Jacobian of the end-effector wrt the
%robot base (link 0)
% input:
%   q: is the joint position vector 1X4
%   L: is the kinematic parameter array (see abbIRB4_params.m)
% return:
%   Jef_0: End-effector Jacobian wrt robot base (6Xn)


% Joint Positions
q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

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
Jef_0 = zeros(6,4);
for i= 1:4
    Jef_0(:, i) = [cross(Z(:,i), (T(:,5) - T(:,i))); Z(:,i)];
end

end