 %% Init
clc
clear
close all

%% Parameter initilization
% Model antrhopomettric data
upper_arm_length = 0.2486838396567298;
forearm_length = 0.25543492290745995;
femur_to_humeris = 0.4552194642124803;
pelvis_to_femur = 0.1429514;

%% Read data
q_values = [[0.8476, 0.8019, 2.3352, 0.8272, 1.5882];
            [0.8782, 0.7805, 1.9735, 0.9422, 1.1801]];

h_values = [[1, 1, 1, 1];
            [1, 1, 1, 1]];

torques = [[1, 1, 1];
           [1, 1, 1]];

%% Solver initilization
g = 9.81;
syms Ugf_U_M Lgf F_M;

epsilon = 1e-6;
Aineq = [-1, 0, 1];
Bineq = -epsilon;
Aeq = [];
Beq = [];
lb = [-5; -5; -5];
ub = [5; 5; 5];

coefficients = extract_coefficients(q_values, h_values, upper_arm_length, forearm_length, g, Ugf_U_M, Lgf, F_M);
humanData = solveForHumanData(coefficients, torques, Aineq, Bineq, Aeq, Beq, lb, ub);

disp(humanData);

%% Function Declarations
function humanData = solveForHumanData(coeffs, torque, Aineq, Bineq, Aeq, Beq, lb, ub)
    % Initialize empty matrices
    A_total = [];
    B_total = [];
    
    % Construct A and B matrices
    for i = 1:size(coeffs, 1)
        A = [coeffs{i, 1}(1), coeffs{i, 1}(2), coeffs{i, 1}(3);
             coeffs{i, 2}(1), coeffs{i, 2}(2), coeffs{i, 2}(3);
             coeffs{i, 3}(1), coeffs{i, 3}(2), coeffs{i, 3}(3)];
        B = [torque(i, 1) - coeffs{i, 1}(4);
             torque(i, 2) - coeffs{i, 2}(4);
             torque(i, 3) - coeffs{i, 3}(4)];
         
        A_total = [A_total; A];
        B_total = [B_total; B];
    end

    A_aug = double(A_total);
    B_aug = double(B_total);
    x0 = [];
    options = optimoptions('lsqlin', 'Display', 'iter');

    humanData = lsqlin(A_aug, B_aug, Aineq, Bineq, Aeq, Beq, lb, ub, x0, options);
end

function coefficients = extract_coefficients(q_values, h_values, U_L, F_L, g, Ugf_U_M, Lgf, F_M)
    % Function to extract coefficients for each set of q values
    % Inputs:
    %   q_values - Array of arrays with each sub-array representing [q1, q21, q31, q4, q5]
    %   T_L, T_W, U_L, F_L, g - Other relevant parameters
    % Outputs:
    %   coefficients - Cell array containing coefficients for each set of q values

    % Initialize output
    coefficients = cell(size(q_values, 1), 3);

    % Iterate through each set of q values
    for i = 1:size(q_values, 1)
        % Extract individual q values from current row
        q1 = q_values(i, 1);
        q21 = q_values(i, 2);
        q31 = q_values(i, 3);
        q4 = q_values(i, 4);
        q5 = q_values(i, 5);
        
        h1 = h_values(i, 1);
        h2 = h_values(i, 2);
        h3 = h_values(i, 3);
        h4 = h_values(i, 4);

        % Compute joint torques using the provided q values
        T_hd = compute_joint_torques(q1, q21, q31, q4, q5, h1, h2, h3, h4, U_L, F_L, g, Ugf_U_M, Lgf, F_M);

        % Extract coefficients for each of the three torques
        [C1, ~] = coeffs(T_hd(1));
        C1 = [C1(3), C1(1), C1(2), 0];
        coefficients{i, 1} = double(C1);

        [C2, ~] = coeffs(T_hd(2));
        C2 = [C2(3), C2(1), C2(2), C2(4)];
        coefficients{i, 2} = double(C2);

        [C3, ~] = coeffs(T_hd(3));
        C3 = [C3(3), C3(1), C3(2), C3(4)];
        coefficients{i, 3} = double(C3);
    end
end

function T_hd = compute_joint_torques(q1, q21, q31, q4, q5, h1, h2, h3, h4, U_L, F_L, g, Ugf_U_M, Lgf, F_M)
    l1 = 0.068;
    l21 = 0.430;
    l22 = 0.446;
    l31 = 0.100;
    l32 = 0.430;
    x1 = 0.05432;
    x21 = 0.21431;
    y21 = 0.01885;
    lg21 = sqrt(x21^2 + y21^2);
    r21 = atan2(y21,x21);
    x22 = 0.33271;
    y22 = 0.0;
    lg22 = sqrt(x22^2 + y22^2);
    r22 = atan2(y22,x22);
    x31 = 0.04632;
    x32 = 0.215;
    
    m1 = 1.51806;
    m21 = 0.25668;
    m22 = 0.55976;
    m31 = 0.09410;
    m32 = 0.14479;
    
    % Transform Points to pelvis frame
    % Declare Fixed Values
    q2_1 = pi/2 - q21;
    q22 = q31 - q2_1 + pi;
    q32=2*pi - (q31 - q2_1);
    alpha_1 = 90;
    alpha_21 = 0;
    alpha_22 = 0;
    alpha_3 = 0;
    alpha_31 = 0;
    alpha_32 = 0;
    alpha_4 = -90;
    alpha_5 = 0;
    alpha_6 = 0;
    alpha_7 = 0;
    l4 = 0.071+0.04;
    l5 = 0.0895;
    l6 = 0.14;
    l7 = F_L;
    
    % Applying human arm dynamic model to find support force
    Lr_f = F_L/2;
    
    if h4 > 80/180*pi
        h4 = 80/180*pi;
    elseif h4 < -80/180*pi
        h4 = -80/180*pi;
    end
    
    t1g = 0;
    t2g = (F_M*(F_L*Lgf*cos(h2)*sin(h4) - U_L*cos(h2) + F_L*Lgf*cos(h4)*sin(h2)*sin(h3)) - U_L*Ugf_U_M*cos(h2))*g;
    t3g = (-F_L*Lgf*F_M*cos(h2)*cos(h3)*cos(h4))*g;
    t4g = (F_M*(F_L*Lgf*cos(h4)*sin(h2) + F_L*Lgf*cos(h2)*sin(h3)*sin(h4)))*g;
    T_s_r = [0 -1  0;
             1  0  0;
             0  0  1];
    J_h_2 = [U_L*cos(h1)*cos(h2) - Lr_f*cos(h4)*(cos(h3)*sin(h1) + cos(h1)*sin(h2)*sin(h3)) - Lr_f*cos(h1)*cos(h2)*sin(h4), Lr_f*sin(h1)*sin(h2)*sin(h4) - U_L*sin(h1)*sin(h2) - Lr_f*cos(h2)*cos(h4)*sin(h1)*sin(h3), -Lr_f*cos(h4)*(cos(h1)*sin(h3) + cos(h3)*sin(h1)*sin(h2)), - Lr_f*sin(h4)*(cos(h1)*cos(h3) - sin(h1)*sin(h2)*sin(h3)) - Lr_f*cos(h2)*cos(h4)*sin(h1); U_L*cos(h2)*sin(h1) + Lr_f*cos(h4)*(cos(h1)*cos(h3) - sin(h1)*sin(h2)*sin(h3)) - Lr_f*cos(h2)*sin(h1)*sin(h4), U_L*cos(h1)*sin(h2) - Lr_f*cos(h1)*sin(h2)*sin(h4) + Lr_f*cos(h1)*cos(h2)*cos(h4)*sin(h3), -Lr_f*cos(h4)*(sin(h1)*sin(h3) - cos(h1)*cos(h3)*sin(h2)), Lr_f*cos(h1)*cos(h2)*cos(h4) - Lr_f*sin(h4)*(cos(h3)*sin(h1) + cos(h1)*sin(h2)*sin(h3)); 0, Lr_f*cos(h2)*sin(h4) - U_L*cos(h2) + Lr_f*cos(h4)*sin(h2)*sin(h3), -Lr_f*cos(h2)*cos(h3)*cos(h4), Lr_f*cos(h4)*sin(h2) + Lr_f*cos(h2)*sin(h3)*sin(h4)];
    Tg_h_2 = [t1g;t2g;t3g;t4g];
    pv_J_h = pinv(J_h_2.');
    F_r2_s = pv_J_h*Tg_h_2;
    F_r2 = T_s_r * F_r2_s;
    
    % Find torque required by each motor
    wl4 = (0.6774) * g;
    F = [F_r2(1);
        F_r2(2);
        wl4+F_r2(3)];
    J = [- l21*cos(q21 - pi/2)*sin(q1) - cos(q21 + q31 + pi/2)*cos(q21 - pi/2)*sin(q1)*(l22 - l31) - sin(q21 + q31 + pi/2)*sin(q1)*sin(q21 - pi/2)*(l22 - l31), -l21*cos(q1)*sin(q21 - pi/2), cos(q21 + q31 + pi/2)*cos(q1)*sin(q21 - pi/2)*(l22 - l31) - sin(q21 + q31 + pi/2)*cos(q1)*cos(q21 - pi/2)*(l22 - l31); l21*cos(q1)*cos(q21 - pi/2) + cos(q21 + q31 + pi/2)*cos(q1)*cos(q21 - pi/2)*(l22 - l31) + sin(q21 + q31 + pi/2)*cos(q1)*sin(q21 - pi/2)*(l22 - l31), -l21*sin(q1)*sin(q21 - pi/2), cos(q21 + q31 + pi/2)*sin(q1)*sin(q21 - pi/2)*(l22 - l31) - sin(q21 + q31 + pi/2)*cos(q21 - pi/2)*sin(q1)*(l22 - l31); 0, -l21*cos(q21 - pi/2), cos(q21 + q31 + pi/2)*cos(q21 - pi/2)*(l22 - l31) + sin(q21 + q31 + pi/2)*sin(q21 - pi/2)*(l22 - l31)];
    JF = (J.')*F;
    
    t1g  = JF(1,1);
    t21g = JF(2,1) + (- m22*(l32*sin(q21 + q31 - (5*pi)/2)*sin(q31) + l32*cos(q21 + q31 - (5*pi)/2)*cos(q31)) - m32*(x32*cos(q21 + q31 - (5*pi)/2)*cos(q31) + x32*sin(q21 + q31 - (5*pi)/2)*sin(q31)) - lg21*m21*cos(r21 - q21 + pi/2))*g;
    t31g = JF(3,1) + (m22*(l31*cos(q31) + lg22*cos(q21 + q31 + r22 + pi/2)*(cos(q21 + q31 - (5*pi)/2)*cos(q31) + sin(q21 + q31 - (5*pi)/2)*sin(q31)) - lg22*sin(q21 + q31 + r22 + pi/2)*(cos(q21 + q31 - (5*pi)/2)*sin(q31) - sin(q21 + q31 - (5*pi)/2)*cos(q31))) + l31*m32*cos(q31) + m31*x31*cos(q31))*g;
    
    T_hd = [t1g,t21g,t31g];
end
