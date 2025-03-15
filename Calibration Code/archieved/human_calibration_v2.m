 %% Init
clc
clear
close all

%% Parameter initilization
% Robot base to pevlis
r_to_p_x = -0.46;
r_to_p_y = -0.02;
r_to_p_z = 0.135;

% Person antrhopomettric data
upper_arm_length = 0.265;
forearm_length = 0.225;
femur_to_humeris = 0.39;
pelvis_to_femur = 0.18;
weight = 75.0;

%% Read data
results_file = 'C:/Users/alexl/Desktop/ARAE_Calibration/Human Testing/subject1/calibration/results.csv';

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

coefficients = extract_coefficients(q_values, femur_to_humeris, pelvis_to_femur, upper_arm_length, forearm_length, g, Ugf_U_M, Lgf, F_M, r_to_p_x, r_to_p_y, r_to_p_z);
humanData = solveForHumanData(coefficients, torques, Aineq, Bineq, Aeq, Beq, lb, ub);

calculated_forearm_mass = humanData(1); % First value: forearm mass
calculated_upper_arm_product = humanData(2); % Second value: upper arm mass * CoM %
calculated_forearm_product = humanData(3); % Third value: forearm mass * CoM %
calculated_forearm_com_fraction = calculated_forearm_product / calculated_forearm_mass;
calculated_humanData = [calculated_forearm_mass;calculated_upper_arm_product;calculated_forearm_product];

% Nominal Values:
nominal_forearm_mass = weight * 1.87/100;
nominal_upper_arm_product = (weight * 3.25/100) * 0.427;
nominal_forearm_com_fraction = 0.417;
nominal_humData = [nominal_forearm_mass; nominal_upper_arm_product; nominal_forearm_mass * nominal_forearm_com_fraction];

nominalTorque = solveForTheoreticalTorque(coefficients, nominal_humData);
personlizedTorque = solveForTheoreticalTorque(coefficients, calculated_humanData);

%% Plot
figure;

num_torque_components = size(torques, 2);

torque_labels = {'Motor Torque 1', 'Motor Torque 2', 'Motor Torque 3'};

for i = 1:num_torque_components
    subplot(3, 1, i);
    
    plot(1:num_positions, torques(:, i), 'b-o', 'LineWidth', 1.5, 'DisplayName', 'Measured Torque');
    hold on;
    plot(1:num_positions, nominalTorque(:, i), 'r--s', 'LineWidth', 1.5, 'DisplayName', 'Nominal Torque');
    plot(1:num_positions, personlizedTorque(:, i), 'g-.d', 'LineWidth', 1.5, 'DisplayName', 'Personlized Torque');
    
    xlabel('Position Index');
    ylabel('Torque (Nm)');
    title(['Comparison of ', torque_labels{i}]);
    legend('Location', 'best');
    grid on;
end

hold off;

actual_values = [nominal_forearm_mass, nominal_upper_arm_product, nominal_forearm_com_fraction];
calculated_values = [calculated_forearm_mass, calculated_upper_arm_product, calculated_forearm_com_fraction];

param_labels = {'Forearm Mass (M_F)', 'Upper Arm Product (MC_U)', 'Forearm COM (COM_F)'};

figure;
bar_data = [actual_values; calculated_values]'; % Transpose for grouped bars
bar(bar_data);

set(gca, 'XTickLabel', param_labels, 'FontSize', 12);
ylabel('Value');
title('Comparison of Nominal and Calibrated Immeasurable Anthropometric Data');
legend({'Actual', 'Calculated'}, 'Location', 'Best');
grid on;

xtips = get(gca,'XTick');
ytips = bar_data; % Heights of bars
for i = 1:numel(xtips)
    text(xtips(i)-0.15, bar_data(i,1), sprintf('%.2f', bar_data(i,1)), 'FontSize', 10, 'VerticalAlignment', 'bottom');
    text(xtips(i)+0.15, bar_data(i,2), sprintf('%.2f', bar_data(i,2)), 'FontSize', 10, 'VerticalAlignment', 'bottom');
end

% Compute percentage error (relative to actual torques)
nominal_percentage_error = (abs(nominalTorque - torques) ./ abs(torques)) * 100;
personalized_percentage_error = (abs(personlizedTorque - torques) ./ abs(torques)) * 100;

% Number of samples
num_samples = size(torques, 1);
sample_indices = 1:num_samples; % X-axis values

% Define colors
colors = {'r', 'b', 'g'}; % Red for Motor 1, Blue for Motor 2, Green for Motor 3

% Plot Percentage Error for Each Motor in One Figure with Three Subplots
figure;
for i = 1:3
    subplot(3,1,i); % Create a subplot for each motor
    hold on;
    
    % Scatter and line for nominal torque
    scatter(sample_indices, nominal_percentage_error(:, i), 50, colors{i}, 'filled');
    plot(sample_indices, nominal_percentage_error(:, i), colors{i}, 'LineWidth', 1.5);
    
    % Scatter and line for personalized torque
    scatter(sample_indices, personalized_percentage_error(:, i), 50, colors{i}, 'o');
    plot(sample_indices, personalized_percentage_error(:, i), '--', 'Color', colors{i}, 'LineWidth', 1.5);
    
    xlabel('Position Index');
    ylabel('Torque Error (%)');
    legend({'Nominal', 'Personalized'}, 'Location', 'best');
    title(['Comparison of Torque Error for ', torque_labels(i)]);
    grid on;

    % Set x-axis ticks to increments of 1
    xticks(1:1:num_samples);
    
    hold off;
end

% Set figure title
sgtitle('Comparison of Torque Error for All Motors');

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

function coefficients = extract_coefficients(q_values, T_L, T_W, U_L, F_L, g, Ugf_U_M, Lgf, F_M, r_to_p_x, r_to_p_y, r_to_p_z)
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
    % Constant declaration
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

    % New shoulder position calculation
    Eproj_r = sqrt(U_L^2 - (Pp_e_x + T_W)^2); % projected radius of elbow position in sagittal plane
    theta_elbow = atan(Pp_e_z/-Pp_e_y);
    L_hip_to_elbow = sqrt(Pp_e_y^2 + Pp_e_z^2);
    theta_elbow_to_torso = acos((T_L^2 + L_hip_to_elbow^2 - Eproj_r^2)/(2*T_L*L_hip_to_elbow));
    y_shoulder = -T_L*cos(theta_elbow + theta_elbow_to_torso);
    z_shoulder = T_L*sin(theta_elbow + theta_elbow_to_torso);
    Pp_s = [-T_W;y_shoulder;z_shoulder];

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
    
    %-----------------------------------------------------------
    % Changed from h1 = atan2(P_e(1)/cos(h2),-P_e(2)/cos(h2))
    % to h1 = atan2(P_e(1),-P_e(2))
    %-----------------------------------------------------------

    U_cal = sqrt(Ps_e(1)^2+Ps_e(2)^2+Ps_e(3)^2);
    h4 = pi/2 - acos((F_L^2 + U_cal^2 - (norm(P_w - P_s))^2)/(2*F_L*U_cal));  % elbow flexion
    h2 = asin(-P_e(3)/U_cal); % shoulder flexion ----X axis
    h1 = atan2(P_e(1),-P_e(2)); % Z axis shoulder abduction
    vf = Ps_w(1)*cos(h1)+Ps_w(2)*sin(h1);
    h3 = atan2(-(Ps_w(1)*sin(h1)*sin(h2)-Ps_w(2)*cos(h1)*sin(h2)+Ps_w(3)*cos(h2)),vf); % shoulder rotation
    
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

    J_h_2 = [U_L*cos(h1)*cos(h2) - Lr_f*cos(h4)*(cos(h3)*sin(h1) + cos(h1)*sin(h2)*sin(h3)) - Lr_f*cos(h1)*cos(h2)*sin(h4), Lr_f*sin(h1)*sin(h2)*sin(h4) - U_L*sin(h1)*sin(h2) - Lr_f*cos(h2)*cos(h4)*sin(h1)*sin(h3), -Lr_f*cos(h4)*(cos(h1)*sin(h3) + cos(h3)*sin(h1)*sin(h2)), -Lr_f*sin(h4)*(cos(h1)*cos(h3) - sin(h1)*sin(h2)*sin(h3)) - Lr_f*cos(h2)*cos(h4)*sin(h1); U_L*cos(h2)*sin(h1) + Lr_f*cos(h4)*(cos(h1)*cos(h3) - sin(h1)*sin(h2)*sin(h3)) - Lr_f*cos(h2)*sin(h1)*sin(h4), U_L*cos(h1)*sin(h2) - Lr_f*cos(h1)*sin(h2)*sin(h4) + Lr_f*cos(h1)*cos(h2)*cos(h4)*sin(h3), -Lr_f*cos(h4)*(sin(h1)*sin(h3) - cos(h1)*cos(h3)*sin(h2)), Lr_f*cos(h1)*cos(h2)*cos(h4) - Lr_f*sin(h4)*(cos(h3)*sin(h1) + cos(h1)*sin(h2)*sin(h3)); 0, Lr_f*cos(h2)*sin(h4) - U_L*cos(h2) + Lr_f*cos(h4)*sin(h2)*sin(h3), -Lr_f*cos(h2)*cos(h3)*cos(h4), Lr_f*cos(h4)*sin(h2) + Lr_f*cos(h2)*sin(h3)*sin(h4)];
    Tg_h_2 = [t1g;t2g;t3g;t4g];
    pv_J_h = pinv(J_h_2.');
    F_r2_s = pv_J_h*Tg_h_2;
    F_r2 = T_s_r * F_r2_s;

    wl4 = (0.6774) * g;
    F = [F_r2(1);
        F_r2(2);
        wl4+F_r2(3)];
    J = [-l21*cos(q21 - pi/2)*sin(q1) - cos(q21 + q31 + pi/2)*cos(q21 - pi/2)*sin(q1)*(l22 - l31) - sin(q21 + q31 + pi/2)*sin(q1)*sin(q21 - pi/2)*(l22 - l31), -l21*cos(q1)*sin(q21 - pi/2), cos(q21 + q31 + pi/2)*cos(q1)*sin(q21 - pi/2)*(l22 - l31) - sin(q21 + q31 + pi/2)*cos(q1)*cos(q21 - pi/2)*(l22 - l31); l21*cos(q1)*cos(q21 - pi/2) + cos(q21 + q31 + pi/2)*cos(q1)*cos(q21 - pi/2)*(l22 - l31) + sin(q21 + q31 + pi/2)*cos(q1)*sin(q21 - pi/2)*(l22 - l31), -l21*sin(q1)*sin(q21 - pi/2), cos(q21 + q31 + pi/2)*sin(q1)*sin(q21 - pi/2)*(l22 - l31) - sin(q21 + q31 + pi/2)*cos(q21 - pi/2)*sin(q1)*(l22 - l31); 0, -l21*cos(q21 - pi/2), cos(q21 + q31 + pi/2)*cos(q21 - pi/2)*(l22 - l31) + sin(q21 + q31 + pi/2)*sin(q21 - pi/2)*(l22 - l31)];
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