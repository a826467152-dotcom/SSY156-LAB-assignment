function [M, C, G] = get_Symbolic_MCG_abbIRB4()
% get_Symbolic_MCG_abbIRB4 Calculates the symbolic dynamic matrices M, C, G
% for the ABB IRB 120 (4DOF) robot.

    %% 1. 定义全部符号变量 (Defining Symbolic Variables)
    % 关节位置与速度 (Joint Positions and Velocities)
    syms q1 q2 q3 q4 real
    syms qp1 qp2 qp3 qp4 real
    Q = [q1; q2; q3; q4];
    Qp = [qp1; qp2; qp3; qp4];
    
    q_size = length(Q);

    % 重力 (Gravity)
    syms gz real
    g_vec = [0; 0; gz];

    % 运动学与动力学参数 (Kinematic and Dynamic Parameters)
    % (为了提取真正的 Yr，这些参数也必须是符号变量)
    syms L1 L2 L7 L8 al L11 L21 L51 L41 real
    syms m1 m2 m3 m4 real
    m = [m1, m2, m3, m4];
    
    % 定义惯性张量元素
    syms I1xx I1xy I1xz I1yy I1yz I1zz real
    syms I2xx I2xy I2xz I2yy I2yz I2zz real
    syms I3xx I3xy I3xz I3yy I3yz I3zz real
    syms I4xx I4xy I4xz I4yy I4yz I4zz real
    
    I_1 = [I1xx, I1xy, I1xz; I1xy, I1yy, I1yz; I1xz, I1yz, I1zz];
    I_2 = [I2xx, I2xy, I2xz; I2xy, I2yy, I2yz; I2xz, I2yz, I2zz];
    I_3 = [I3xx, I3xy, I3xz; I3xy, I3yy, I3yz; I3xz, I3yz, I3zz];
    I_4 = [I4xx, I4xy, I4xz; I4xy, I4yy, I4yz; I4xz, I4yz, I4zz];
    I = {I_1, I_2, I_3, I_4};

    %% 2. 运动学准备 (Kinematics Preparation)
    theta = [q1; q2 - pi/2; q3 + al; q4 - al + pi/2];
    d = [L1; 0; 0; 0];
    alpha = [-pi/2; 0; 0; 0];
    a = [0; L2; L7; L8];
    
    theta_cm = [q1; q2 - pi/2; q3 + al; q4 - al + pi/2];
    d_cm = [L11; 0; 0; 0];
    alpha_cm = [0; 0; 0; 0];
    a_cm = [0; L21; L51; L41];
    
    T_std = get_DH_Transforms_Sym(theta, d, alpha, a);
    Tcm_rel = get_DH_Transforms_Sym(theta_cm, d_cm, alpha_cm, a_cm);
    
    T1_0 = T_std{1};
    T2_0 = T1_0 * T_std{2};
    T3_0 = T2_0 * T_std{3};
    
    Tcm1_0 = Tcm_rel{1};
    Tcm2_0 = T1_0 * Tcm_rel{2};
    Tcm3_0 = T2_0 * Tcm_rel{3};
    Tcm4_0 = T3_0 * Tcm_rel{4};
    Tcm = {Tcm1_0, Tcm2_0, Tcm3_0, Tcm4_0};
    
    Jcm1 = get_Jacobian_Sym(q_size, {Tcm1_0});
    Jcm2 = get_Jacobian_Sym(q_size, {T1_0, Tcm2_0});
    Jcm3 = get_Jacobian_Sym(q_size, {T1_0, T2_0, Tcm3_0});
    Jcm4 = get_Jacobian_Sym(q_size, {T1_0, T2_0, T3_0, Tcm4_0});
    Jcm = {Jcm1, Jcm2, Jcm3, Jcm4};

    %% 3. 惯性矩阵 M (Inertia Matrix)
    M = sym(zeros(q_size, q_size)); % 必须初始化为符号零矩阵
    for idx = 1:q_size
        R_i = Tcm{idx}(1:3, 1:3); 
        J_v = Jcm{idx}(1:3, :);
        J_w = Jcm{idx}(4:6, :);
        M = M + m(idx) * (J_v.' * J_v) + J_w.' * (R_i * I{idx} * R_i.') * J_w;
    end
    M = simplify(M); % 简化矩阵以加快后续求导

    %% 4. 离心力与科里奥利力矩阵 C (Coriolis Matrix - 解析求导法)
    C = sym(zeros(q_size, q_size)); % 必须初始化为符号零矩阵
    for i = 1:q_size
        for j = 1:q_size
            for k = 1:q_size
                % 使用真正的符号偏导数 diff() 计算 Christoffel 符号
                dM_ij_qk = diff(M(i,j), Q(k));
                dM_ik_qj = diff(M(i,k), Q(j));
                dM_jk_qi = diff(M(j,k), Q(i));
                
                c_ijk = 0.5 * (dM_ij_qk + dM_ik_qj - dM_jk_qi);
                C(i, j) = C(i, j) + c_ijk * Qp(k);
            end
        end
    end
    C = simplify(C);

    %% 5. 重力矩阵 G (Gravitational Matrix)
    G = sym(zeros(q_size, 1)); % 必须初始化为符号零矩阵
    for k = 1:q_size
        J_v = Jcm{k}(1:3, :);
        G = G + J_v.' * (m(k) * g_vec); 
    end
    G = simplify(G);

end

%% 辅助函数 (必须修改为支持符号矩阵初始化)
function T = get_DH_Transforms_Sym(theta, d, alpha, a)
    num_frames = length(theta);
    T = cell(num_frames, 1);
    for i = 1:num_frames
        T{i} = [cos(theta(i)), -sin(theta(i)) * cos(alpha(i)),  sin(theta(i)) * sin(alpha(i)), a(i) * cos(theta(i));
                sin(theta(i)),  cos(theta(i)) * cos(alpha(i)), -cos(theta(i)) * sin(alpha(i)), a(i) * sin(theta(i));
                0,           sin(alpha(i)),              cos(alpha(i)),             d(i);
                0,           0,                       0,                      1];
    end
end

function Jef = get_Jacobian_Sym(q_size, T_in)
    j = length(T_in); 
    Z = sym(zeros(3, j+1)); % 关键修复：初始化为符号数组
    P = sym(zeros(3, j+1)); 
    Z(:, 1) = [0; 0; 1];
    P(:, 1) = [0; 0; 0];
    
    for i = 1:j
        Z(:, i+1) = T_in{i}(1:3, 3); 
        P(:, i+1) = T_in{i}(1:3, 4); 
    end
    
    Jef = sym(zeros(6, q_size)); % 关键修复：初始化为符号数组
    P_ef = P(:, j+1); 
    for i = 1:j
        Jef(1:3, i) = cross(Z(:, i), (P_ef - P(:, i)));
        Jef(4:6, i) = Z(:, i);
    end
end