 %% Init
clc
clear
close all

%% Parameter initilization
% Robot base to pevlis
r_to_p_x = -0.385;
r_to_p_y = 0.275;
r_to_p_z = 0.1335;

% Model antrhopomettric data
upper_arm_length = 0.2486838396567298;
forearm_length = 0.25543492290745995;
femur_to_humeris = 0.4552194642124803;
pelvis_to_femur = 0.1429514;

%% Read data
results_file = 'C:/Users/alexl/Desktop/ARAE_Calibration/MuJoCo Simulation/results/with_hand_mass_20p_no_IDC/results.csv';
json_filename = 'C:/Users/alexl/Desktop/ARAE_Calibration/MuJoCo Simulation/simulation_data.json';

opts = detectImportOptions(results_file);
opts.DataLines = [2 Inf];

% Read the data from the file
data = readtable(results_file, opts);

positions = unique(data{:, 1});
num_positions = numel(positions);

% Preallocate cell arrays to store q_values and torques
q_values = [];
torques = [];

for i = 1:num_positions
    position_data = data(i, :);
    q_values = [q_values; position_data{:, 2:6}];
    torques = [torques; position_data{:, 7:9}]; 
end

%% Solver initilization
g = 9.81;
syms Ugf_U_M Lgf F_M;

epsilon = 1e-6;
Aineq = [-1, 0, 1];
Bineq = -epsilon;
Aeq = [];
Beq = [];
lb = [0; 0; 0];
ub = [5; 5; 5];
lambda = 1e-4; 

coefficients = extract_coefficients(q_values, femur_to_humeris, pelvis_to_femur, upper_arm_length, forearm_length, g, Ugf_U_M, Lgf, F_M, r_to_p_x, r_to_p_y, r_to_p_z);
humanData = solveForHumanData(coefficients, torques, Aineq, Bineq, Aeq, Beq, lb, ub, lambda);

calculated_forearm_mass = humanData(1); % First value: forearm mass
calculated_upper_arm_product = humanData(2); % Second value: upper arm mass * CoM %
calculated_forearm_product = humanData(3); % Third value: forearm mass * CoM %
calculated_forearm_com_fraction = calculated_forearm_product / calculated_forearm_mass;
disp("Calculated:")
disp(calculated_forearm_mass)
disp(calculated_upper_arm_product)
disp(calculated_forearm_com_fraction)

% Actual Values:
actual_forearm_mass = 0.93874053;
actual_upper_arm_product = 1.0542687374296 * 0.57370836881454366;
actual_forearm_com_fraction = 0.4974308358852518;
actual_humData = [actual_forearm_mass; actual_upper_arm_product; actual_forearm_com_fraction];

disp("Actual:")
disp(actual_forearm_mass)
disp(actual_upper_arm_product)
disp(actual_forearm_com_fraction)

error_forearm_mass = abs(calculated_forearm_mass - actual_forearm_mass) / actual_forearm_mass * 100;
error_upper_arm_product = abs(calculated_upper_arm_product - actual_upper_arm_product) / actual_upper_arm_product * 100;
error_forearm_com_fraction = abs(calculated_forearm_com_fraction - actual_forearm_com_fraction) / actual_forearm_com_fraction * 100;

theoreticalTorque = solveForTheoreticalTorque(coefficients, actual_humData);
disp("Torque from simulation:")
disp(torques)
disp("Theoretical Torque:")
disp(theoreticalTorque)

% Graph
errors = [error_forearm_mass; error_upper_arm_product; error_forearm_com_fraction]';
error_labels = {'Forearm Mass', 'Upper Arm Product', 'Forearm COM Fraction'};
sample_indices = 1:length(error_forearm_mass);
figure;
bar(sample_indices, errors, 'grouped'); % Grouped bar chart
grid on;
set(gca, 'XTick', sample_indices, 'XTickLabel', string(sample_indices));
set(gca, 'TickLength', [0 0]); % Remove tick marks for cleaner appearance
xtickangle(45);
title('Error Analysis of Biomechanical Variables', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('Sample Index', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Error (%)', 'FontSize', 12, 'FontWeight', 'bold');
legend(error_labels, 'Location', 'northeast', 'FontSize', 10);
set(gca, 'FontSize', 12);
ylim([0, max(errors, [], 'all') + 5]); % Add padding to y-axis limits

%% Function Declarations
function theoreticalTorque = solveForTheoreticalTorque(coeffs, humData)
theoreticalTorque = [];
    for i = 1:size(coeffs,1)
        A = [coeffs{i,1}(1), coeffs{i,1}(2), coeffs{i,1}(3);
             coeffs{i,2}(1), coeffs{i,2}(2), coeffs{i,2}(3);
             coeffs{i,3}(1), coeffs{i,3}(2), coeffs{i,3}(3)];
        B = [coeffs{i,1}(4);
             coeffs{i,2}(4);
             coeffs{i,3}(4)];
        torque = A * humData + B;
        theoreticalTorque = [theoreticalTorque; torque'];
    end
end

function humanData = solveForHumanData(coeffs, torque, Aineq, Bineq, Aeq, Beq, lb, ub, lambda)
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

function coefficients = extract_coefficients(q_values, T_L, T_W, U_L, F_L, g, Ugf_U_M, Lgf, F_M, r_to_p_x, r_to_p_y, r_to_p_z)
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

        % Compute joint torques using the provided q values
        T_hd = compute_joint_torques(q1, q21, q31, q4, q5, T_L, T_W, U_L, F_L, g, Ugf_U_M, Lgf, F_M, r_to_p_x, r_to_p_y, r_to_p_z);

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

function T_hd = compute_joint_torques(q1, q21, q31, q4, q5, T_L, T_W, U_L, F_L, g, Ugf_U_M, Lgf, F_M, r_to_p_x, r_to_p_y, r_to_p_z)
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
    
    % convert to radians
    alpha_1 = deg2rad(alpha_1);
    alpha_21 = deg2rad(alpha_21);
    alpha_22= deg2rad(alpha_22);
    alpha_3 = deg2rad(alpha_3);
    alpha_31= deg2rad(alpha_31);
    alpha_32= deg2rad(alpha_32);
    alpha_4 = deg2rad(alpha_4);
    alpha_5 = deg2rad(alpha_5);
    alpha_6 = deg2rad(alpha_6);
    alpha_7 = deg2rad(alpha_7);
    
    % DH parameters
    a1 = 0;
    d1 = l1;
    a21 = l21;
    d21 = 0;
    a3 = l22-l31;
    d3 = 0;
    a4 = 0;
    d4 = -l4;
    a5 = l5;
    d5 = 0.04;  % the defination between mocap and robot should be different
                % height from cuff surface to the center of forearm (adjustable)
    a6 = l6;
    d6 = 0;
    a7 = l7;
    d7 = 0;
    a22 = l22;
    d22 = 0;
    a31 = l31;
    d31 = 0;
    a32 =l32;
    d32 = 0;
    
    % DH parameters matrix "table"
    PT = [q1 alpha_1 a1 d1;
          q2_1 alpha_21 a21 d21;
          q22 alpha_22 a3 d3;
          q4 alpha_4 a4 d4;
          q5 alpha_5 a5 d5;
          pi/2 alpha_6 a6 d6;
          pi alpha_7 a7 d7];
    
    % Transformation Matrix
    i=1;
    H0_1 = [cos(PT(i,1)) -sin(PT(i,1))*cos(PT(i,2)) sin(PT(i,1))*sin(PT(i,2)) PT(i,3)*cos(PT(i,1));
            sin(PT(i,1)) cos(PT(i,1))*cos(PT(i,2)) -cos(PT(i,1))*sin(PT(i,2)) PT(i,3)*sin(PT(i,1));
            0 sin(PT(i,2)) cos(PT(i,2)) PT(i,4);
            0 0 0 1];
    i=2;
    H1_21 = [cos(PT(i,1)) -sin(PT(i,1))*cos(PT(i,2)) sin(PT(i,1))*sin(PT(i,2)) PT(i,3)*cos(PT(i,1));
            sin(PT(i,1)) cos(PT(i,1))*cos(PT(i,2)) -cos(PT(i,1))*sin(PT(i,2)) PT(i,3)*sin(PT(i,1));
            0 sin(PT(i,2)) cos(PT(i,2)) PT(i,4);
            0 0 0 1];
    i=3;
    H21_22 = [cos(PT(i,1)) -sin(PT(i,1))*cos(PT(i,2)) sin(PT(i,1))*sin(PT(i,2)) PT(i,3)*cos(PT(i,1));
            sin(PT(i,1)) cos(PT(i,1))*cos(PT(i,2)) -cos(PT(i,1))*sin(PT(i,2)) PT(i,3)*sin(PT(i,1));
            0 sin(PT(i,2)) cos(PT(i,2)) PT(i,4);
            0 0 0 1];
    i=4;
    H22_4 = [cos(PT(i,1)) -sin(PT(i,1))*cos(PT(i,2)) sin(PT(i,1))*sin(PT(i,2)) PT(i,3)*cos(PT(i,1));
            sin(PT(i,1)) cos(PT(i,1))*cos(PT(i,2)) -cos(PT(i,1))*sin(PT(i,2)) PT(i,3)*sin(PT(i,1));
            0 sin(PT(i,2)) cos(PT(i,2)) PT(i,4);
            0 0 0 1];
    i=5;
    H4_5 = [cos(PT(i,1)) -sin(PT(i,1))*cos(PT(i,2)) sin(PT(i,1))*sin(PT(i,2)) PT(i,3)*cos(PT(i,1));
            sin(PT(i,1)) cos(PT(i,1))*cos(PT(i,2)) -cos(PT(i,1))*sin(PT(i,2)) PT(i,3)*sin(PT(i,1));
            0 sin(PT(i,2)) cos(PT(i,2)) PT(i,4);
            0 0 0 1];
    i=6;
    H5_6 = [cos(PT(i,1)) -sin(PT(i,1))*cos(PT(i,2)) sin(PT(i,1))*sin(PT(i,2)) PT(i,3)*cos(PT(i,1));
            sin(PT(i,1)) cos(PT(i,1))*cos(PT(i,2)) -cos(PT(i,1))*sin(PT(i,2)) PT(i,3)*sin(PT(i,1));
            0 sin(PT(i,2)) cos(PT(i,2)) PT(i,4);
            0 0 0 1];
    i=7;
    H6_7 = [cos(PT(i,1)) -sin(PT(i,1))*cos(PT(i,2)) sin(PT(i,1))*sin(PT(i,2)) PT(i,3)*cos(PT(i,1));
            sin(PT(i,1)) cos(PT(i,1))*cos(PT(i,2)) -cos(PT(i,1))*sin(PT(i,2)) PT(i,3)*sin(PT(i,1));
            0 sin(PT(i,2)) cos(PT(i,2)) PT(i,4);
            0 0 0 1];
    
    % transfer matrix from robot base to points
    H0_4 = H0_1 * H1_21 * H21_22 * H22_4;
    H0_5= H0_1 * H1_21 * H21_22 * H22_4 * H4_5;
    H0_6= H0_1 * H1_21 * H21_22 * H22_4 * H4_5 * H5_6;
    H0_7= H0_1 * H1_21 * H21_22 * H22_4 * H4_5 * H5_6 * H6_7;
    
    % postition vector from base to end-effector
    P0_0= [0; 0; 0; 1];
    P0_4 = H0_4 * P0_0;
    P0_5= H0_5 * P0_0;
    x0 = P0_5(1,1);
    y0 = P0_5(2,1);
    z0 = P0_5(3,1);
    
    % Transformation matrix from robot frame to pelvis frame
    Tr_p = Trans(r_to_p_x,r_to_p_y,r_to_p_z)*Rotz(-pi/2);
    
    % Transforming robot base points to pelvis base points
    Pp_5 = Tr_p * P0_5;
    
    P0_e = H0_6 * P0_0;
    Pp_e = Tr_p * P0_e;
    
    P0_w = H0_7 * P0_0;
    Pp_w = Tr_p * P0_w;
    
    % Transform to shoulder frame
    % Finding shoulder point in pelvis frame
    Pp_e_x = Pp_e(1);
    Pp_e_y = Pp_e(2);
    Pp_e_z = Pp_e(3);
    
    Eproj_r = sqrt(U_L^2 - (Pp_e_x - T_W)^2); % projected radius of elbow position in sagittal plane
    l_H_Eproj = sqrt(Pp_e_y^2 + Pp_e_z^2); % distance between projected elbow joint and hip joint in sagittal plane
    cal_HS = Pp_e_z + Eproj_r;
    
    if cal_HS >= T_L
        amend_Eproj_r = Eproj_r;
        [yout,zout] = circcirc(0,0,T_L,Pp_e_y,Pp_e_z,amend_Eproj_r);
        Pp_s = [T_W;yout(2);zout(2)];
     else
        amend_Eproj_r = T_L-Pp_e_z; % Forcing the projected radius+Pe(z) is equal to l_SH
        [yout,zout] = circcirc(0,0,T_L,Pp_e_y,Pp_e_z,amend_Eproj_r);
        Pp_s = [T_W;yout(2);zout(2)];
    end
    
    y1 = 0;
    z1 = 0;
    r1 = T_L;
    y2 = Pp_e_y;
    z2 = Pp_e_z;
    r2 = amend_Eproj_r;
    
    d = sqrt((y2 - y1)^2 + (z2 - z1)^2);
    
    a = (r1^2 - r2^2 + d^2) / (2 * d);
    h = sqrt(r1^2 - a^2);
    
    y2m = y1 + a * (y2 - y1) / d;
    z2m = z1 + a * (z2 - z1) / d;
    
    yout = y2m + h * (y2 - y1) / d;
    zout = z2m - h * (z2 - z1) / d;
    
    Pp_s = [T_W;yout;zout];
    
    % Transforming pelvis base points to shoulder base points
    Ps_e = Trans(-Pp_s(1),-Pp_s(2),-Pp_s(3))*Pp_e;
    Ps_w = Trans(-Pp_s(1),-Pp_s(2),-Pp_s(3))*Pp_w;
    
    % Find human joint angles
    P_w = [Ps_w(1);
           Ps_w(2);
           Ps_w(3)];
    P_e = [Ps_e(1);
           Ps_e(2);
           Ps_e(3)];
    P_s = [0;
           0;
           0];
    
    U_cal = sqrt(Ps_e(1)^2+Ps_e(2)^2+Ps_e(3)^2);
    h4 = pi/2 - acos((F_L^2 + U_cal^2 - (norm(P_w - P_s))^2)/(2*F_L*U_cal));  % elbow flexion
    h2 = asin(-P_e(3)/U_cal); % shoulder flexion ----X axis
    h1 = atan2(P_e(1)/cos(h2),-P_e(2)/cos(h2)); % Z axis shoulder abduction
    vf = Ps_w(1)*cos(h1)+Ps_w(2)*sin(h1);
    h3 = atan2(-(Ps_w(1)*sin(h1)*sin(h2)-Ps_w(2)*cos(h1)*sin(h2)+Ps_w(3)*cos(h2))/(F_L*cos(h4)),vf/(F_L*cos(h4))); % shoulder rotation
    
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

% Helper function for DH transformation
function H = dh_matrix(param)
    theta = param(1);
    alpha = param(2);
    a = param(3);
    d = param(4);
    H = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
         sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0, sin(alpha), cos(alpha), d;
         0, 0, 0, 1];
end

% Translation matrix
function T = Trans(x, y, z)
    T = [1 0 0 x;
         0 1 0 y;
         0 0 1 z;
         0 0 0 1];
end

% Rotation matrix about z-axis
function Rz = Rotz(angle)
    Rz = [cos(angle), -sin(angle), 0, 0;
          sin(angle), cos(angle), 0, 0;
          0, 0, 1, 0;
          0, 0, 0, 1];
end