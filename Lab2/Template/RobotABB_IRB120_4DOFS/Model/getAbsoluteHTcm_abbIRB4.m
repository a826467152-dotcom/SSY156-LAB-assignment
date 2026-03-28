function [HTcm_0, HTcm_W] = getAbsoluteHTcm_abbIRB4(q, L, T0_W)
%GETABSOLUTEHTCM calculates the absolute Homogeneous transformations of
% the cms wrt the robot base link0 and the wcf
% input:
%   q: is the joint position vector 1X4
%   T0_W: transformation (4x4) of the robot base wrt to wcf
% return:
%   HTcm_0: List with all the absolute CM HT (4x4) wrt robot base (0)
%   HTcm_W: List with all the absolute CM HT (4x4) wrt wcf (w)

% Joint postion

q1=q(1);
q2=q(2);
q3=q(3);
q4=q(4);

% Kinematic Paramters
%    1  2   3  4  5  6   7   8   9  10  11
% p=[L1,L2,L7,L8,al,L11,L21,L31,L32,L41,L51];

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

ca23=cos(al + q2 + q3);
sa23=sin(al + q2 + q3);

s234=sin(q2 + q3 + q4);
c234=cos(q2 + q3 + q4);


% Absolute Homogeneous Transformations
% TODO: Define the absolute HT wrt robot base (link 0)
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



%List with all the HT wrt to link 0, starting with H0_0
HTcm_0={eye(4),Tcm1_0, Tcm2_0, Tcm3_0, Tcm4_0};

%TODO: Create a list with all the HT wrt to the wcf (w), starting with
%Hw_w. In total, HT_W should have 6 elements Hw_w, H0_w, ..., Hef_w.
Tcm1_w = T0_W * Tcm1_0;
Tcm2_w = T0_W * Tcm2_0;
Tcm3_w = T0_W * Tcm3_0;
Tcm4_w = T0_W * Tcm4_0;

HTcm_W={eye(4), T0_W, Tcm1_w, Tcm2_w, Tcm3_w, Tcm4_w};


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