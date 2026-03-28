function Jefp=Jp_EF_abbIRB4(q,qp,L)
% Jp_EF_abbIRB4 Computes the time derivative of the EF's geometric Jacobian

% Joint Position
q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

% Joint Velocity
qp1=qp(1);
qp2=qp(2);
qp3=qp(3);
qp4=qp(4);

% Kinematic Parameters
L1=L(1);
L2=L(2);
L7=L(3);
L8=L(4);
al=L(5);

% Common substitutions
c1=cos(q1);
s1=sin(q1);

c2=cos(q2);
s2=sin(q2);

c234=cos(q2 + q3 + q4);
s234=sin(q2 + q3 + q4);

ca23=cos(al + q2 + q3);
sa23=sin(al + q2 + q3);

e3=L8*s234;
e4=L8*c234;
e1=L7*ca23 - e3;
e2=e4 + L7*sa23;


d1=L2*s2 + e2;
d2=L2*c2 + e1;
d3=e3 - L7*ca23;

% TODO: compute the time derivative of the ef's geometric jacobian. You can
% use the chain rule to compute it, i.e. 
% Jp=\sum_{i=1}^{n}\frac{\partial Jef}{\partial q_i}\dot{q_i}
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

Z = zeros(3, 5);
T = zeros(3, 5);

Z(:, 2:5) = [T1_0(1:3,3), T2_0(1:3,3), T3_0(1:3,3), T4_0(1:3,3)];
Z(:, 1) = [0; 0; 1];
T(:, 2:5) = [T1_0(1:3,4), T2_0(1:3,4), T3_0(1:3,4), T4_0(1:3,4)];
Jef_0 = zeros(6,4);
for i= 1:4
    Jef_0(:, i) = [cross(Z(:,i), (T(:,5) - T(:,i))); Z(:,i)];
end

% Partial derivative Jef wrt q1

dJ_dq = zeros(6, 4, 4);
for i = 1:4
    for j = 1:4
        if j <= i
            dJv = cross(Z(:, j), Jef_0(1:3, i));
            dJw = cross(Z(:, j), Z(:, i));
        else
            dJv = cross(Z(:, i), Jef_0(1:3, j));
            dJw = [0; 0; 0];
        end
        dJ_dq(:, i, j) = [dJv; dJw];
    end
end

Jefp = reshape( reshape(dJ_dq, 24, 4) * qp(:), 6, 4 );

end


