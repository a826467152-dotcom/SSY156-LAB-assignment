function [HT_0, HT_W] = getAbsoluteHT_abbIRB4(q,L, T0_W)
%GETABSOLUTEHT_abbIRB4 calculates the absolute Homogeneous transformations wrt the
%robot base link0 and the wcf
% input:
%   q: is the joint position vector 1X4
%   L: is the kinematic parameter array (see abbIRB4_params.m)
%   T0_W: transformation (4x4) of the robot base wrt to wcf
% return:
%   HT_0: Cell array with all the absolute HT (4x4) wrt robot base
%   HT_W: Cell array with all the absolute HT (4x4) wrt wcf

% Joint postions

q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

% Kinematic Paramters
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

%ca23=cos(al + q2 + q3);
%sa23=sin(al + q2 + q3);

%s234=sin(q2 + q3 + q4);
%c234=cos(q2 + q3 + q4);


% Absolute Homogeneous Transformations
% TODO: Define the absolute HT wrt robot base (link 0)
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

% List with all the HT wrt to link 0, starting with H0_0
HT_0={eye(4),T1_0, T2_0, T3_0, T4_0};

%TODO: Create a list with all the HT wrt to the wcf (w), starting with
%Hw_w. In total, HT_W should have 6 elements Hw_w, H0_w, ..., Hef_w.
T1_w = T0_W * T1_0;
T2_w = T0_W * T2_0;
T3_w = T0_W * T3_0;
T4_w = T0_W * T4_0;

HT_W={eye(4),T0_W, T1_w, T2_w, T3_w, T4_w};


end

