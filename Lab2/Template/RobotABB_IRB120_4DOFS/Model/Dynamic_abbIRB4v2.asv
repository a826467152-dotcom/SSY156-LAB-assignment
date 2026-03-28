function Qpp=Dynamic_abbIRB4v2(u)
% Dynamic_abbIRB4 Calculates the direct dynamics of robot ABB IRB 120 (4DOF)
%
% This model uses external parameters


%Joint Position
q1=u(1);
q2=u(2);
q3=u(3);
q4=u(4);

%Joint Velocity
qp1=u(5);
qp2=u(6);
qp3=u(7);
qp4=u(8);

%Gravity Vector
gz=u(9);

%Viscous Friction Matrix
Beta(1,1)=u(10);
Beta(2,2)=u(11);
Beta(3,3)=u(12);
Beta(4,4)=u(13);

%Time
t=u(14);


%Joint Position Vector
Q=[q1; q2; q3; q4];

%Joint Velocity Vector
Qp=[qp1; qp2; qp3; qp4];

% Load Robot Parameters
abb_irb120_params

% Inertia Tensors
I111=I1(1);
I112=I1(2);
I113=I1(3);
I122=I1(4);
I123=I1(5);
I133=I1(6);

I211=I2(1);
I212=I2(2);
I213=I2(3);
I222=I2(4);
I223=I2(5);
I233=I2(6);

I311=I3(1);
I312=I3(2);
I313=I3(3);
I322=I3(4);
I323=I3(5);
I333=I3(6);

I411=I4(1);
I412=I4(2);
I413=I4(3);
I422=I4(4);
I423=I4(5);
I433=I4(6);

%Common substitutions
c2=cos(q2);
s2=sin(q2);

ca23=cos(al + q2 + q3);
sa23=sin(al + q2 + q3);

c234=cos(q2 + q3 + q4);
s234=sin(q2 + q3 + q4);

ca3=cos(al + q3);
sa3=sin(al + q3);

c2_234=cos(2*q2 + 2*q3 + 2*q4);
s2_234=sin(2*q2 + 2*q3 + 2*q4);

c34=cos(q3 + q4);
s34=sin(q3 + q4);

c2_a23=cos(2*al + 2*q2 + 2*q3);
s2_a23=sin(2*al + 2*q2 + 2*q3);

cam4=cos(al - q4);
sam4=sin(al - q4);

c22=cos(2*q2);
s22=sin(2*q2);

ca_223_4=cos(al + 2*q2 + 2*q3 + q4);

c22_34=cos(2*q2 + q3 + q4);

%% Preparation
m = [m1, m2, m3, m4];

% 2. 严格对齐 Symbolic 隐式逻辑的惯性张量映射
I_1 = [I1(1), I1(2), I1(3); 
       I1(2), I1(4), I1(5); 
       I1(3), I1(5), I1(6)];

I_2 = [I2(1), I2(2), I2(3); 
       I2(2), I2(4), I2(5); 
       I2(3), I2(5), I2(6)];

I_3 = [I3(1), I3(2), I3(3); 
       I3(2), I3(4), I3(5); 
       I3(3), I3(5), I3(6)];

I_4 = [I4(1), I4(2), I4(3); 
       I4(2), I4(4), I4(5); 
       I4(3), I4(5), I4(6)];
I = {I_1, I_2, I_3, I_4};

%% Preparation
theta = [q1; q2 - pi/2; q3 + al; q4 - al + pi/2];
d = [L1; 0; 0; 0];
alpha = [-pi/2; 0; 0; 0];
a = [0; L2; L7; L8];

theta_cm = [q1; q2 - pi/2; q3 + al; q4 - al + pi/2];
d_cm = [L11; 0; 0; 0];
alpha_cm = [0; 0; 0; 0];
a_cm = [0; L21; L51; L41];

T_std = get_DH_Transforms_from_Table(theta, d, alpha, a);
Tcm_rel = get_DH_Transforms_from_Table(theta_cm, d_cm, alpha_cm, a_cm);

T1_0 = T_std{1};
T2_0 = T1_0 * T_std{2};
T3_0 = T2_0 * T_std{3};

Tcm1_0 = Tcm_rel{1};
Tcm2_0 = T1_0 * Tcm_rel{2};
Tcm3_0 = T2_0 * Tcm_rel{3};
Tcm4_0 = T3_0 * Tcm_rel{4};
Tcm = {Tcm1_0, Tcm2_0, Tcm3_0, Tcm4_0};

q_size = length(Q);

Jcm1 = get_Jacobian_from_DH_Transforms(q_size, {Tcm1_0});
Jcm2 = get_Jacobian_from_DH_Transforms(q_size, {T1_0, Tcm2_0});
Jcm3 = get_Jacobian_from_DH_Transforms(q_size, {T1_0, T2_0, Tcm3_0});
Jcm4 = get_Jacobian_from_DH_Transforms(q_size, {T1_0, T2_0, T3_0, Tcm4_0});
Jcm = {Jcm1, Jcm2, Jcm3, Jcm4};

%% Inertia Matrix
M = zeros(q_size, q_size);
for idx = 1:q_size
    R_i = Tcm{idx}(1:3, 1:3); 
    J_v = Jcm{idx}(1:3, :);
    J_w = Jcm{idx}(4:6, :);
    % 严格使用 .' 以保证解析兼容性
    M = M + m(idx) * (J_v.' * J_v) + J_w.' * (R_i * I{idx} * R_i.') * J_w;
end

%% Centripetal and Coriolis Matrix (复步求导法)
delta = 1e-8; % 极小的虚部扰动量
dM_dq = zeros(q_size, q_size, q_size); 
for k = 1:q_size
    Q_c = Q; 
    Q_c(k) = Q_c(k) + 1i * delta; 
    M_c = compute_M_fast(Q_c, L1, L2, L7, L8, al, L11, L21, L51, L41, m, I);
    dM_dq(:, :, k) = imag(M_c) / delta; 
end

C = zeros(q_size, q_size);
for idx_i = 1:q_size
    for idx_j = 1:q_size
        for idx_k = 1:q_size
            c_ijk = 0.5 * (dM_dq(idx_i, idx_j, idx_k) + dM_dq(idx_i, idx_k, idx_j) - dM_dq(idx_j, idx_k, idx_i));
            C(idx_i, idx_j) = C(idx_i, idx_j) + c_ijk * Qp(idx_k);
        end
    end
end

%% Gravitational Torques Vector
G = zeros(q_size, 1);
g_vec = [0; 0; gz]; 
for k = 1:q_size
    J_v = Jcm{k}(1:3, :);
    G = G + J_v.' * (m(k) * g_vec); 
end

Tau = zeros(4,1);
Qpp = M \ (Tau - C * Qp - G - Beta * Qp);

end

%% External Functions
function T = get_DH_Transforms_from_Table(theta, d, alpha, a)
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
    j = length(T_in); 
    Z = zeros(3, j+1);
    P = zeros(3, j+1); 
    Z(:, 1) = [0; 0; 1];
    P(:, 1) = [0; 0; 0];
    for i = 1:j
        Z(:, i+1) = T_in{i}(1:3, 3); 
        P(:, i+1) = T_in{i}(1:3, 4); 
    end
    Jef = zeros(6, q_size);
    P_ef = P(:, j+1); 
    for i = 1:j
        Jef(:, i) = [cross(Z(:, i), (P_ef - P(:, i))); Z(:, i)];
    end
end

function M_out = compute_M_fast(q_val, L1, L2, L7, L8, al, L11, L21, L51, L41, m, I)
    th = [q_val(1); q_val(2) - pi/2; q_val(3) + al; q_val(4) - al + pi/2];
    d_val = [L1; 0; 0; 0];
    alp = [-pi/2; 0; 0; 0];
    a_val = [0; L2; L7; L8];
    
    th_cm = [q_val(1); q_val(2) - pi/2; q_val(3) + al; q_val(4) - al + pi/2];
    d_cm = [L11; 0; 0; 0];
    alp_cm = [0; 0; 0; 0];
    a_cm = [0; L21; L51; L41];
    
    T_std = get_DH_Transforms_from_Table(th, d_val, alp, a_val);
    Tcm_rel = get_DH_Transforms_from_Table(th_cm, d_cm, alp_cm, a_cm);
    
    T1_0 = T_std{1}; 
    T2_0 = T1_0 * T_std{2}; 
    T3_0 = T2_0 * T_std{3};
    
    Tcm1_0 = Tcm_rel{1}; 
    Tcm2_0 = T1_0 * Tcm_rel{2}; 
    Tcm3_0 = T2_0 * Tcm_rel{3}; 
    Tcm4_0 = T3_0 * Tcm_rel{4};
    Tcm = {Tcm1_0, Tcm2_0, Tcm3_0, Tcm4_0};
    
    q_len = length(q_val);
    J1 = get_Jacobian_from_DH_Transforms(q_len, {Tcm1_0});
    J2 = get_Jacobian_from_DH_Transforms(q_len, {T1_0, Tcm2_0});
    J3 = get_Jacobian_from_DH_Transforms(q_len, {T1_0, T2_0, Tcm3_0});
    J4 = get_Jacobian_from_DH_Transforms(q_len, {T1_0, T2_0, T3_0, Tcm4_0});
    J_all = {J1, J2, J3, J4};
    
    M_out = zeros(q_len, q_len);
    for idx = 1:q_len
        R_i = Tcm{idx}(1:3, 1:3);
        J_v = J_all{idx}(1:3, :);
        J_w = J_all{idx}(4:6, :);
        M_out = M_out + m(idx) * (J_v.' * J_v) + J_w.' * (R_i * I{idx} * R_i.') * J_w;
    end
end