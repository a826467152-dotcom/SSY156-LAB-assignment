function Yr_out = Yr_abbIRB4(u)
%YR_ABBIRB4 Computes the Robot regressor

% Joint Position
q1=u(1);
q2=u(2);
q3=u(3);
q4=u(4);


% Joint Velocity
qp1=u(5);
qp2=u(6);
qp3=u(7);
qp4=u(8);


% Joint Velocity reference
qp1r=u(9);
qp2r=u(10);
qp3r=u(11);
qp4r=u(12);


% Joint Acceleration reference
qpp1r=u(13);
qpp2r=u(14);
qpp3r=u(15);
qpp4r=u(16);

%Gravity
gz=u(17);

% Kinematic Parameters
L=abbIRB4_params;

% Angle offset
al=L(5);

% Common substitutions
c22=cos(2*q2);
s22=sin(2*q2);

c2a_22_23=cos(2*al + 2*q2 + 2*q3);
s2a_22_23=sin(2*al + 2*q2 + 2*q3);

sa_22_3=sin(al + 2*q2 + q3);
ca_22_3=cos(al + 2*q2 + q3);

ca3=cos(al + q3);
sa3=sin(al + q3);

s22_23_24=sin(2*q2 + 2*q3 + 2*q4);
c22_23_24=cos(2*q2 + 2*q3 + 2*q4);

c22_3_4=cos(2*q2 + q3 + q4);
s22_3_4=sin(2*q2 + q3 + q4);

s34=sin(q3 + q4);
c34=cos(q3 + q4);

c2=cos(q2);
s2=sin(q2);

sa_22_23_4=sin(al + 2*q2 + 2*q3 + q4);
ca_22_23_4=cos(al + 2*q2 + 2*q3 + q4);

ca23=cos(al + q2 + q3);
sa23=sin(al + q2 + q3);

sam4=sin(al - q4);
cam4=cos(al - q4);

c234=cos(q2 + q3 + q4);
s234=sin(q2 + q3 + q4);

% TODO: Define the robot regressor Yr(q,qp,qpr,qppr)
Yr(1,1) = qpp1r;
Yr(1,2) = -cos(q2)*(qp1*qp2r*sin(q2) - qpp1r*cos(q2) + qp2*qp1r*sin(q2));
Yr(1,3) = - qpp1r*sin(2*q2) - qp1*qp2r*cos(2*q2) - qp2*qp1r*cos(2*q2);
Yr(1,4) = qpp2r*cos(q2) - qp2*qp2r*sin(q2);
Yr(1,5) = (qp1*qp2r*sin(2*q2))/2 - qpp1r*(cos(2*q2)/2 - 1/2) + (qp2*qp1r*sin(2*q2))/2;
Yr(1,6) = - qpp2r*sin(q2) - qp2*qp2r*cos(q2);
Yr(1,7) = 0;
Yr(1,8) = qpp1r/2 + (qpp1r*cos(2*al + 2*q2 + 2*q3))/2 - (qp1*qp2r*sin(2*al + 2*q2 + 2*q3))/2 - (qp2*qp1r*sin(2*al + 2*q2 + 2*q3))/2 - (qp1*qp3r*sin(2*al + 2*q2 + 2*q3))/2 - (qp3*qp1r*sin(2*al + 2*q2 + 2*q3))/2;
Yr(1,9) = - qpp1r*sin(2*al + 2*q2 + 2*q3) - qp1*qp2r*cos(2*al + 2*q2 + 2*q3) - qp2*qp1r*cos(2*al + 2*q2 + 2*q3) - qp1*qp3r*cos(2*al + 2*q2 + 2*q3) - qp3*qp1r*cos(2*al + 2*q2 + 2*q3);
Yr(1,10) = qpp2r*cos(al + q2 + q3) + qpp3r*cos(al + q2 + q3) - qp2*qp2r*sin(al + q2 + q3) - qp2*qp3r*sin(al + q2 + q3) - qp3*qp2r*sin(al + q2 + q3) - qp3*qp3r*sin(al + q2 + q3);
Yr(1,11) = qpp1r/2 - (qpp1r*cos(2*al + 2*q2 + 2*q3))/2 + (qp1*qp2r*sin(2*al + 2*q2 + 2*q3))/2 + (qp2*qp1r*sin(2*al + 2*q2 + 2*q3))/2 + (qp1*qp3r*sin(2*al + 2*q2 + 2*q3))/2 + (qp3*qp1r*sin(2*al + 2*q2 + 2*q3))/2;
Yr(1,12) = - qpp2r*sin(al + q2 + q3) - qpp3r*sin(al + q2 + q3) - qp2*qp2r*cos(al + q2 + q3) - qp2*qp3r*cos(al + q2 + q3) - qp3*qp2r*cos(al + q2 + q3) - qp3*qp3r*cos(al + q2 + q3);
Yr(1,13) = 0;
Yr(1,14) = qpp1r/2 - (qpp1r*cos(2*q2 + 2*q3 + 2*q4))/2 + (qp1*qp2r*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp2*qp1r*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp1*qp3r*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp3*qp1r*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp1*qp4r*sin(2*q2 + 2*q3 + 2*q4))/2 + (qp4*qp1r*sin(2*q2 + 2*q3 + 2*q4))/2;
Yr(1,15) = qpp1r*sin(2*q2 + 2*q3 + 2*q4) + qp1*qp2r*cos(2*q2 + 2*q3 + 2*q4) + qp2*qp1r*cos(2*q2 + 2*q3 + 2*q4) + qp1*qp3r*cos(2*q2 + 2*q3 + 2*q4) + qp3*qp1r*cos(2*q2 + 2*q3 + 2*q4) + qp1*qp4r*cos(2*q2 + 2*q3 + 2*q4) + qp4*qp1r*cos(2*q2 + 2*q3 + 2*q4);
Yr(1,16) = - qpp2r*sin(q2 + q3 + q4) - qpp3r*sin(q2 + q3 + q4) - qpp4r*sin(q2 + q3 + q4) - qp2*qp2r*cos(q2 + q3 + q4) - qp2*qp3r*cos(q2 + q3 + q4) - qp3*qp2r*cos(q2 + q3 + q4) - qp2*qp4r*cos(q2 + q3 + q4) - qp3*qp3r*cos(q2 + q3 + q4) - qp4*qp2r*cos(q2 + q3 + q4) - qp3*qp4r*cos(q2 + q3 + q4) - qp4*qp3r*cos(q2 + q3 + q4) - qp4*qp4r*cos(q2 + q3 + q4);
Yr(1,17) = qpp1r/2 + (qpp1r*cos(2*q2 + 2*q3 + 2*q4))/2 - (qp1*qp2r*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp2*qp1r*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp1*qp3r*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp3*qp1r*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp1*qp4r*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp4*qp1r*sin(2*q2 + 2*q3 + 2*q4))/2;
Yr(1,18) = qp2*qp2r*sin(q2 + q3 + q4) - qpp3r*cos(q2 + q3 + q4) - qpp4r*cos(q2 + q3 + q4) - qpp2r*cos(q2 + q3 + q4) + qp2*qp3r*sin(q2 + q3 + q4) + qp3*qp2r*sin(q2 + q3 + q4) + qp2*qp4r*sin(q2 + q3 + q4) + qp3*qp3r*sin(q2 + q3 + q4) + qp4*qp2r*sin(q2 + q3 + q4) + qp3*qp4r*sin(q2 + q3 + q4) + qp4*qp3r*sin(q2 + q3 + q4) + qp4*qp4r*sin(q2 + q3 + q4);
Yr(1,19) = 0;
Yr(1,20) = qp1r;
Yr(1,21) = 0;
Yr(1,22) = 0;
Yr(1,23) = 0;
Yr(1,24) = 0;
Yr(1,25) = 0;
Yr(1,26) = 0;
Yr(1,27) = 0;
Yr(1,28) = 0;
Yr(1,29) = 0;
Yr(1,30) = (qp1*qp2r*sin(2*q2))/2 - qpp1r*(cos(2*q2)/2 - 1/2) + (qp2*qp1r*sin(2*q2))/2;
Yr(1,31) = (qp1*qp2r*sin(2*q2))/2 - qpp1r*(cos(2*q2)/2 - 1/2) + (qp2*qp1r*sin(2*q2))/2;
Yr(1,32) = qpp1r/2 - (qpp1r*cos(2*al + 2*q2 + 2*q3))/2 + (qp1*qp2r*sin(2*al + 2*q2 + 2*q3))/2 + (qp2*qp1r*sin(2*al + 2*q2 + 2*q3))/2 + (qp1*qp3r*sin(2*al + 2*q2 + 2*q3))/2 + (qp3*qp1r*sin(2*al + 2*q2 + 2*q3))/2;
Yr(1,33) = (qp1*qp2r*sin(2*q2))/2 - qpp1r*(cos(2*q2)/2 - 1/2) + (qp2*qp1r*sin(2*q2))/2;
Yr(1,34) = qpp1r/2 + (qpp1r*cos(2*q2 + 2*q3 + 2*q4))/2 - (qp1*qp2r*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp2*qp1r*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp1*qp3r*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp3*qp1r*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp1*qp4r*sin(2*q2 + 2*q3 + 2*q4))/2 - (qp4*qp1r*sin(2*q2 + 2*q3 + 2*q4))/2;
Yr(1,35) = qpp1r/2 - (qpp1r*cos(2*al + 2*q2 + 2*q3))/2 + (qp1*qp2r*sin(2*al + 2*q2 + 2*q3))/2 + (qp2*qp1r*sin(2*al + 2*q2 + 2*q3))/2 + (qp1*qp3r*sin(2*al + 2*q2 + 2*q3))/2 + (qp3*qp1r*sin(2*al + 2*q2 + 2*q3))/2;
Yr(1,36) = qpp1r*cos(al + q3) - qpp1r*cos(al + 2*q2 + q3) + qp1*qp2r*sin(al + 2*q2 + q3) + qp2*qp1r*sin(al + 2*q2 + q3) + (qp1*qp3r*sin(al + 2*q2 + q3))/2 + (qp3*qp1r*sin(al + 2*q2 + q3))/2 - (qp1*qp3r*sin(al + q3))/2 - (qp3*qp1r*sin(al + q3))/2;
Yr(1,37) = qpp1r*sin(2*q2 + q3 + q4) - qpp1r*sin(q3 + q4) + qp1*qp2r*cos(2*q2 + q3 + q4) + qp2*qp1r*cos(2*q2 + q3 + q4) + (qp1*qp3r*cos(2*q2 + q3 + q4))/2 + (qp3*qp1r*cos(2*q2 + q3 + q4))/2 + (qp1*qp4r*cos(2*q2 + q3 + q4))/2 + (qp4*qp1r*cos(2*q2 + q3 + q4))/2 - (qp1*qp3r*cos(q3 + q4))/2 - (qp3*qp1r*cos(q3 + q4))/2 - (qp1*qp4r*cos(q3 + q4))/2 - (qp4*qp1r*cos(q3 + q4))/2;
Yr(1,38) = qpp1r*sin(al + 2*q2 + 2*q3 + q4) + qpp1r*sin(al - q4) - (qp1*qp4r*cos(al - q4))/2 - (qp4*qp1r*cos(al - q4))/2 + qp1*qp2r*cos(al + 2*q2 + 2*q3 + q4) + qp2*qp1r*cos(al + 2*q2 + 2*q3 + q4) + qp1*qp3r*cos(al + 2*q2 + 2*q3 + q4) + qp3*qp1r*cos(al + 2*q2 + 2*q3 + q4) + (qp1*qp4r*cos(al + 2*q2 + 2*q3 + q4))/2 + (qp4*qp1r*cos(al + 2*q2 + 2*q3 + q4))/2;
Yr(1,39) = qpp1r*cos(al + q3) - qpp1r*cos(al + 2*q2 + q3) + qp1*qp2r*sin(al + 2*q2 + q3) + qp2*qp1r*sin(al + 2*q2 + q3) + (qp1*qp3r*sin(al + 2*q2 + q3))/2 + (qp3*qp1r*sin(al + 2*q2 + q3))/2 - (qp1*qp3r*sin(al + q3))/2 - (qp3*qp1r*sin(al + q3))/2;
Yr(2,1) = 0;
Yr(2,2) = (qp1*qp1r*sin(2*q2))/2;
Yr(2,3) = qp1*qp1r*cos(2*q2);
Yr(2,4) = qpp1r*cos(q2);
Yr(2,5) = -(qp1*qp1r*sin(2*q2))/2;
Yr(2,6) = -qpp1r*sin(q2);
Yr(2,7) = qpp2r;
Yr(2,8) = (qp1*qp1r*sin(2*al + 2*q2 + 2*q3))/2;
Yr(2,9) = qp1*qp1r*cos(2*al + 2*q2 + 2*q3);
Yr(2,10) = qpp1r*cos(al + q2 + q3);
Yr(2,11) = -(qp1*qp1r*sin(2*al + 2*q2 + 2*q3))/2;
Yr(2,12) = -qpp1r*sin(al + q2 + q3);
Yr(2,13) = qpp2r + qpp3r;
Yr(2,14) = -(qp1*qp1r*sin(2*q2 + 2*q3 + 2*q4))/2;
Yr(2,15) = -qp1*qp1r*cos(2*q2 + 2*q3 + 2*q4);
Yr(2,16) = -qpp1r*sin(q2 + q3 + q4);
Yr(2,17) = (qp1*qp1r*sin(2*q2 + 2*q3 + 2*q4))/2;
Yr(2,18) = -qpp1r*cos(q2 + q3 + q4);
Yr(2,19) = qpp2r + qpp3r + qpp4r;
Yr(2,20) = 0;
Yr(2,21) = qp2r;
Yr(2,22) = 0;
Yr(2,23) = 0;
Yr(2,24) = -gz*sin(q2);
Yr(2,25) = -gz*sin(q2);
Yr(2,26) = -gz*sin(al + q2 + q3);
Yr(2,27) = -gz*sin(q2);
Yr(2,28) = -gz*cos(q2 + q3 + q4);
Yr(2,29) = -gz*sin(al + q2 + q3);
Yr(2,30) = qpp2r - (qp1*qp1r*sin(2*q2))/2;
Yr(2,31) = qpp2r - (qp1*qp1r*sin(2*q2))/2;
Yr(2,32) = qpp2r + qpp3r - (qp1*qp1r*sin(2*al + 2*q2 + 2*q3))/2;
Yr(2,33) = qpp2r - (qp1*qp1r*sin(2*q2))/2;
Yr(2,34) = qpp2r + qpp3r + qpp4r + (qp1*qp1r*sin(2*q2 + 2*q3 + 2*q4))/2;
Yr(2,35) = qpp2r + qpp3r - (qp1*qp1r*sin(2*al + 2*q2 + 2*q3))/2;
Yr(2,36) = 2*qpp2r*cos(al + q3) + qpp3r*cos(al + q3) - qp1*qp1r*sin(al + 2*q2 + q3) - qp2*qp3r*sin(al + q3) - qp3*qp2r*sin(al + q3) - qp3*qp3r*sin(al + q3);
Yr(2,37) = - 2*qpp2r*sin(q3 + q4) - qpp3r*sin(q3 + q4) - qpp4r*sin(q3 + q4) - qp1*qp1r*cos(2*q2 + q3 + q4) - qp2*qp3r*cos(q3 + q4) - qp3*qp2r*cos(q3 + q4) - qp2*qp4r*cos(q3 + q4) - qp3*qp3r*cos(q3 + q4) - qp4*qp2r*cos(q3 + q4) - qp3*qp4r*cos(q3 + q4) - qp4*qp3r*cos(q3 + q4) - qp4*qp4r*cos(q3 + q4);
Yr(2,38) = 2*qpp2r*sin(al - q4) + 2*qpp3r*sin(al - q4) + qpp4r*sin(al - q4) - qp2*qp4r*cos(al - q4) - qp4*qp2r*cos(al - q4) - qp3*qp4r*cos(al - q4) - qp4*qp3r*cos(al - q4) - qp4*qp4r*cos(al - q4) - qp1*qp1r*cos(al + 2*q2 + 2*q3 + q4);
Yr(2,39) = 2*qpp2r*cos(al + q3) + qpp3r*cos(al + q3) - qp1*qp1r*sin(al + 2*q2 + q3) - qp2*qp3r*sin(al + q3) - qp3*qp2r*sin(al + q3) - qp3*qp3r*sin(al + q3);
Yr(3,1) = 0;
Yr(3,2) = 0;
Yr(3,3) = 0;
Yr(3,4) = 0;
Yr(3,5) = 0;
Yr(3,6) = 0;
Yr(3,7) = 0;
Yr(3,8) = (qp1*qp1r*sin(2*al + 2*q2 + 2*q3))/2;
Yr(3,9) = qp1*qp1r*cos(2*al + 2*q2 + 2*q3);
Yr(3,10) = qpp1r*cos(al + q2 + q3);
Yr(3,11) = -(qp1*qp1r*sin(2*al + 2*q2 + 2*q3))/2;
Yr(3,12) = -qpp1r*sin(al + q2 + q3);
Yr(3,13) = qpp2r + qpp3r;
Yr(3,14) = -(qp1*qp1r*sin(2*q2 + 2*q3 + 2*q4))/2;
Yr(3,15) = -qp1*qp1r*cos(2*q2 + 2*q3 + 2*q4);
Yr(3,16) = -qpp1r*sin(q2 + q3 + q4);
Yr(3,17) = (qp1*qp1r*sin(2*q2 + 2*q3 + 2*q4))/2;
Yr(3,18) = -qpp1r*cos(q2 + q3 + q4);
Yr(3,19) = qpp2r + qpp3r + qpp4r;
Yr(3,20) = 0;
Yr(3,21) = 0;
Yr(3,22) = qp3r;
Yr(3,23) = 0;
Yr(3,24) = 0;
Yr(3,25) = 0;
Yr(3,26) = -gz*sin(al + q2 + q3);
Yr(3,27) = 0;
Yr(3,28) = -gz*cos(q2 + q3 + q4);
Yr(3,29) = -gz*sin(al + q2 + q3);
Yr(3,30) = 0;
Yr(3,31) = 0;
Yr(3,32) = qpp2r + qpp3r - (qp1*qp1r*sin(2*al + 2*q2 + 2*q3))/2;
Yr(3,33) = 0;
Yr(3,34) = qpp2r + qpp3r + qpp4r + (qp1*qp1r*sin(2*q2 + 2*q3 + 2*q4))/2;
Yr(3,35) = qpp2r + qpp3r - (qp1*qp1r*sin(2*al + 2*q2 + 2*q3))/2;
Yr(3,36) = qpp2r*cos(al + q3) - (qp1*qp1r*sin(al + 2*q2 + q3))/2 + (qp1*qp1r*sin(al + q3))/2 + qp2*qp2r*sin(al + q3);
Yr(3,37) = (qp1*qp1r*cos(q3 + q4))/2 - (qp1*qp1r*cos(2*q2 + q3 + q4))/2 - qpp2r*sin(q3 + q4) + qp2*qp2r*cos(q3 + q4);
Yr(3,38) = 2*qpp2r*sin(al - q4) + 2*qpp3r*sin(al - q4) + qpp4r*sin(al - q4) - qp2*qp4r*cos(al - q4) - qp4*qp2r*cos(al - q4) - qp3*qp4r*cos(al - q4) - qp4*qp3r*cos(al - q4) - qp4*qp4r*cos(al - q4) - qp1*qp1r*cos(al + 2*q2 + 2*q3 + q4);
Yr(3,39) = qpp2r*cos(al + q3) - (qp1*qp1r*sin(al + 2*q2 + q3))/2 + (qp1*qp1r*sin(al + q3))/2 + qp2*qp2r*sin(al + q3);
Yr(4,1) = 0;
Yr(4,2) = 0;
Yr(4,3) = 0;
Yr(4,4) = 0;
Yr(4,5) = 0;
Yr(4,6) = 0;
Yr(4,7) = 0;
Yr(4,8) = 0;
Yr(4,9) = 0;
Yr(4,10) = 0;
Yr(4,11) = 0;
Yr(4,12) = 0;
Yr(4,13) = 0;
Yr(4,14) = -(qp1*qp1r*sin(2*q2 + 2*q3 + 2*q4))/2;
Yr(4,15) = -qp1*qp1r*cos(2*q2 + 2*q3 + 2*q4);
Yr(4,16) = -qpp1r*sin(q2 + q3 + q4);
Yr(4,17) = (qp1*qp1r*sin(2*q2 + 2*q3 + 2*q4))/2;
Yr(4,18) = -qpp1r*cos(q2 + q3 + q4);
Yr(4,19) = qpp2r + qpp3r + qpp4r;
Yr(4,20) = 0;
Yr(4,21) = 0;
Yr(4,22) = 0;
Yr(4,23) = qp4r;
Yr(4,24) = 0;
Yr(4,25) = 0;
Yr(4,26) = 0;
Yr(4,27) = 0;
Yr(4,28) = -gz*cos(q2 + q3 + q4);
Yr(4,29) = 0;
Yr(4,30) = 0;
Yr(4,31) = 0;
Yr(4,32) = 0;
Yr(4,33) = 0;
Yr(4,34) = qpp2r + qpp3r + qpp4r + (qp1*qp1r*sin(2*q2 + 2*q3 + 2*q4))/2;
Yr(4,35) = 0;
Yr(4,36) = 0;
Yr(4,37) = (qp1*qp1r*cos(q3 + q4))/2 - (qp1*qp1r*cos(2*q2 + q3 + q4))/2 - qpp2r*sin(q3 + q4) + qp2*qp2r*cos(q3 + q4);
Yr(4,38) = qpp2r*sin(al - q4) + qpp3r*sin(al - q4) + (qp1*qp1r*cos(al - q4))/2 + qp2*qp2r*cos(al - q4) + qp2*qp3r*cos(al - q4) + qp3*qp2r*cos(al - q4) + qp3*qp3r*cos(al - q4) - (qp1*qp1r*cos(al + 2*q2 + 2*q3 + q4))/2;
Yr(4,39) = 0;

% TODO: Define the robot regressor extensio with the viscous friction
% coefficients

% TODO: Extended regressor with robot dynamic model and viscous friction

% Regressor serialization
% TODO: define the variables n=DOFs, p=number of parameters
n=4;
p=39;
Yr_out=reshape(Yr,n*p,1);

%To recover the Yr matrix
%Yr_in=reshape(Yr_out,n,p);
end

