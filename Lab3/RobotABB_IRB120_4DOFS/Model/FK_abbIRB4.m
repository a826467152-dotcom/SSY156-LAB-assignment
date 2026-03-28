function X=FK_abbIRB4(q,L)
% FK_abbIRB4 Calculates the Forward Kinematics of the EF (Linear Positions)
    % FK_abbIRB4 Calculates the Forward Kinematics of the EF (Linear Positions)
    
    % Joint Positions
    q1=q(1);
    q2=q(2);
    q3=q(3);
    q4=q(4);
    
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
    
    % Forward Kinematics EF wrt to base link 0 (3x1 only positions)
    % TODO: define the FK model 3x1
    % 计算机械臂在 XY 平面上的水平总投影长度 (Radius)
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
    
    T4_0 = T{1} * T{2} * T{3} * T{4};
    
    X = T4_0(1:3,4);
% Use the obtained function from previous asssignments
end