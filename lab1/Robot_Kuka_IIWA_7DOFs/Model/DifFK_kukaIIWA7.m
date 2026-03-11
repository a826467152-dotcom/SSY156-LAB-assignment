function Xp=DifFK_kukaIIWA7(q,qp,L)
%DIFFK_KUKAIIWA7 calculates the end-effector's linear and angular velocity
%vector wrt the robot base (link 0)
% input:
%   q: is the joint position vector 1X7
%   qp: is the joint velocity vector 1X7
%   L: is the kinematic parameter array (see abbIRB4_params.m)
% return:
%   Xp: End-effector linear and angular velocitites wrt robot base (6X1)


% Joint positions
q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);
q5=q(5);
q6=q(6);
q7=q(7);

% Joint positions
qp1=qp(1);
qp2=qp(2);
qp3=qp(3);
qp4=qp(4);
qp5=qp(5);
qp6=qp(6);
qp7=qp(7);

% Kinematic parameters
L1=L(1);
L2=L(2);
L3=L(3);
L4=L(4);

% Common Substitutions
s1=sin(q1);
s2=sin(q2);
s3=sin(q3);
s4=sin(q4);
s5=sin(q5);
s6=sin(q6);

c1=cos(q1);
c2=cos(q2);
c3=cos(q3);
c4=cos(q4);
c5=cos(q5);
c6=cos(q6);

s12=s1*s2;
s13=s1*s3;
s14=s1*s4;
s15=s1*s5;
s16=s1*s6;

s23=s2*s3;
s24=s2*s4;

s34=s3*s4;
s35=s3*s5;
s36=s3*s6;

s45=s4*s5;

s56=s5*s6;

c12=c1*c2;
c13=c1*c3;
c14=c1*c4;

c23=c2*c3;
c24=c2*c4;
c25=c2*c5;
c26=c2*c6;

c34=c3*c4;
c35=c3*c5;
c36=c3*c6;

c45=c4*c5;
c46=c4*c6;

c56=c5*c6;

c123=c12*c3;
c12s3=c12*s3;
c1s23=c1*s24;
c14s2=c14*s2;
c23s1=c23*s1;
c34s2=c34*s2;
c4s12=c4*s12;
c2s13=c2*s13;
s124=s12*s4;
c46s2=c46*s2;
c236s4=c23*c6*s4;
c3s24=c3*s24;
c5s246=c5*s24*s6;
c2345s6=c23*c45*s6;
c6s34=c6*s34;
c26s4=c26*s4;
c245s6=c24*c5*s6;
c5s23=c5*s23;
c2s45=c2*s45;
c24s6=c24*s6;
c256s4=c25*c6*s4;
c6s235=c6*s23*s5;
c3456s2=c34*c56*s2;
c136s4=c13*c6*s4;
c13s4=c13*s4;
c1s356=c1*s35*s6;
c26s134=c26*s13*s4;
c1345s6=c13*c45*s6;
c36s14=c36*s14;
s1356=s13*s56;
c245s136=c24*c5*s13*s6;
c345s16=c34*c5*s16;
c35s1=c35*s1;
c125s3=c12*c5*s3;
c4s135=c4*s13*s5;
c25s13=c25*s13;
c135=c13*c5;
c14s35=c14*s35;
c234s15=c23*c4*s15;
s135=s23*s5;

% Define the absolute HT wrt robot base (link 0)
theta = [q1 ;q2 ;q3; q4;q5 ;q6 ; q7];
d = [L1; 0; L2; 0; L3; 0; L4];
alpha = [-pi/2; pi/2; pi/2; -pi/2; -pi/2; pi/2; 0];
a = [0; 0; 0; 0; 0; 0; 0];

T = cell(7,1);

length = size(theta,1);
for i = 1:length
    T{i} = [cos(theta(i)), -sin(theta(i)) * cos(alpha(i)),  sin(theta(i)) * sin(alpha(i)), a(i) * cos(theta(i));
            sin(theta(i)),  cos(theta(i)) * cos(alpha(i)), -cos(theta(i)) * sin(alpha(i)), a(i) * sin(theta(i));
            0, sin(alpha(i)), cos(alpha(i)), d(i);
            0, 0, 0, 1];
end
% TODO: Define the absolute HT wrt robot base (link 0)
T1_0=T{1};

T2_0=T1_0 * T{2};

T3_0=T2_0 * T{3};

T4_0=T3_0 * T{4};

T5_0=T4_0 * T{5};

T6_0=T5_0 * T{6};

T7_0=T6_0 * T{7};

% TODO: define the Jacobian of the ef wrt robot base (link 0) 6xn
Z = zeros(3, 8);
T = zeros(3, 8);

Z(:, 2:8) = [T1_0(1:3,3), T2_0(1:3,3), T3_0(1:3,3), T4_0(1:3,3), T5_0(1:3,3), T6_0(1:3,3), T7_0(1:3,3)];
Z(:, 1) = [0; 0; 1];
T(:, 2:8) = [T1_0(1:3,4), T2_0(1:3,4), T3_0(1:3,4), T4_0(1:3,4), T5_0(1:3,4), T6_0(1:3,4), T7_0(1:3,4)];
Jef_0 = zeros(6,7);
for i= 1:7
    Jef_0(:, i) = [cross(Z(:,i), (T(:,8) - T(:,i))); Z(:,i)];
end

% TODO: define the Jacobian of the ef wrt robot base (link 0) 6xn
Jef = Jef_0;

%TODO: Calcualte the linear and angular velocity vector of the ef wrt 0
Xp= Jef * [qp1; qp2; qp3; qp4; qp5; qp6; qp7];
