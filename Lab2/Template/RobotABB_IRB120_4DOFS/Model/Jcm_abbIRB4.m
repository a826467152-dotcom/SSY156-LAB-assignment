function [Jcm1,Jcm2,Jcm3,Jcm4] = Jcm_abbIRB4(q,L)
%JCM CM geometric Jacobians
%   q: Joint position vector 4x1
%   L: Dynamic parameters, see abbIRB4_dyn_params.m

% Joint Positions
q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

% Kinematic and dynamic parameters
L1=L(1);
L2=L(2);
L7=L(3);
L8=L(4);
al=L(5);
L11=L(6);
L21=L(7);
L41=L(10);
L51=L(11);

% Common substitutions
c1=cos(q1);
s1=sin(q1);

c2=cos(q2);
s2=sin(q2);

c234=cos(q2 + q3 + q4);
s234=sin(q2 + q3 + q4);

ca23=cos(al + q2 + q3);
sa23=sin(al + q2 + q3);

% Jacobians CM
%TODO: define the geometric Jacobians for each CM

L8=L(4);
theta = [q1; q2 - pi/2; q3 + al; q4 - al + pi/2];
d = [L1; 0; 0; 0];
alpha = [-pi/2; 0; 0; 0];
a = [0; L2; L7; L8];

theta_cm = [q1; q2 - pi/2; q3 + al; q4 - al + pi/2];
d_cm = [L11; 0; 0; 0];
alpha_cm = [0; 0; 0; 0];
a_cm = [0; L21; L51; L41];

T = get_DH_Transforms_from_Table(theta, d, alpha, a);
Tcm = get_DH_Transforms_from_Table(theta_cm, d_cm, alpha_cm, a_cm);

T1_0 = T{1};
T2_0 = T1_0 * T{2};
T3_0 = T2_0 * T{3};

Tcm1_0 = Tcm{1};


Tcm2_0 = T1_0 * Tcm{2};


Tcm3_0 = T2_0 * Tcm{3};


Tcm4_0 = T3_0 * Tcm{4};

q_size = length(q);

Jcm1=get_Jacobian_from_DH_Transforms(q_size, {Tcm1_0});
				
Jcm2=get_Jacobian_from_DH_Transforms(q_size, {T1_0, Tcm2_0});
				
Jcm3=get_Jacobian_from_DH_Transforms(q_size, {T1_0, T2_0, Tcm3_0});

Jcm4=get_Jacobian_from_DH_Transforms(q_size, {T1_0, T2_0, T3_0, Tcm4_0});

end

function T = get_DH_Transforms_from_Table(theta, d, alpha, a)
    % GET_DH_TRANSFORMS_FROM_TABLE Calculates DH transformation matrices from a DH table.
    %
    % Input:
    %   DH_table - N x 4 matrix. Each row contains the DH parameters for one link.
    %              Column order: [theta, d, a, alpha]
    %
    % Output:
    %   T        - N x 1 cell array containing the homogeneous transformation matrices H_{i}^{i-1}
    num_frames = size(theta, 1);
    
    T = cell(num_frames, 1);

    for i = 1:num_frames
        
        T{i} = [cos(theta(i)), -sin(theta(i)) * cos(alpha(i)),  sin(theta(i)) * sin(alpha(i)), a(i) * cos(theta(i));
                sin(theta(i)),  cos(theta(i)) * cos(alpha(i)), -cos(theta(i)) * sin(alpha(i)), a(i) * sin(theta(i));
                0,           sin(alpha(i)),              cos(alpha(i)),             d(i);
                0,           0,                       0,                      1];
    end
end

function Jef = get_Jacobian_from_DH_Transforms(q_size, T_in)
    % GET_JACOBIAN_FROM_DH_TRANSFORMS calculates the geometric Jacobian
    % Input: T_in - Cell array of ABSOLUTE transformation matrices (H_i^0)
    % Output: Jef - 6xN Geometric Jacobian matrix
    
    j = length(T_in); % Number of joints
    
    Z = zeros(3, j+1);
    P = zeros(3, j+1); % 'P' represents the Origin position of each frame
    
    % Base frame (Frame 0) properties wrt itself
    Z(:, 1) = [0; 0; 1];
    P(:, 1) = [0; 0; 0];
    
    % Extract Z-axes and Origins from absolute HTs
    for i = 1:j
        Z(:, i+1) = T_in{i}(1:3, 3); % Z_i axis
        P(:, i+1) = T_in{i}(1:3, 4); % O_i position
    end
    
    Jef = zeros(6, q_size);
    
    % The actual End-Effector position is the last column in P
    P_ef = P(:, j+1); 
    
    % Construct the Jacobian
    for i = 1:j
        % i-th column of Jacobian. 
        % Z(:,i) is Z_{i-1}. P(:,i) is O_{i-1}.
        Jef(:, i) = [cross(Z(:, i), (P_ef - P(:, i))); Z(:, i)];
    end
end
