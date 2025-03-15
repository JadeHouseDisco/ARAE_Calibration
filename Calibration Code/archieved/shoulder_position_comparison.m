 %% Init
clc
clear
close all

%% Parameter initilization
% Robot base to pevlis
r_to_p_x = -0.385;
r_to_p_y = 0.275;
r_to_p_z = 0.1335;

% Person antrhopomettric data
upper_arm_length = 0.2486838396567298;
forearm_length = 0.25543492290745995;
femur_to_humeris = 0.4552194642124803;
pelvis_to_femur = 0.1429514;

%% Read Data
results_file = 'C:/Users/alexl/Desktop/ARAE_Calibration/MuJoCo Simulation/results/fixed_with_position/results.csv';

opts = detectImportOptions(results_file);
opts.DataLines = [2 Inf];

% Read the data from the file
data = readtable(results_file, opts);

positions = unique(data{:, 1});
num_positions = numel(positions);

% Preallocate arrays
q_values = zeros(num_positions, 5);
torques = zeros(num_positions, 3);
shoulder_positions_actual = zeros(num_positions, 3);
shoulder_positions_adjusted = zeros(num_positions, 3);
shoulder_positions_calculated = zeros(num_positions, 3);
shoulder_positions_calculated_v2 = zeros(num_positions, 3);

% Extract Data
for i = 1:num_positions
    position_data = data(i, :);
    q_values(i, :) = position_data{:, 2:6};  % Extract joint angles
    torques(i, :) = position_data{:, 7:9};  % Extract torque values
    shoulder_positions_actual(i, :) = position_data{:, 10:12};  % Extract actual shoulder position
end

% Adjust the Frame of Reference for Actual Shoulder Positions
for i = 1:num_positions
    old_x = shoulder_positions_actual(i, 1);
    old_y = shoulder_positions_actual(i, 2);
    old_z = shoulder_positions_actual(i, 3);
    
    shoulder_positions_adjusted(i, :) = [old_y, -old_x, old_z]; % Apply transformation
end

%% Compute Shoulder Position using Function
for i = 1:num_positions
    % Extract joint angles for this position
    q1 = q_values(i, 1);
    q21 = q_values(i, 2);
    q31 = q_values(i, 3);
    q4 = q_values(i, 4);
    q5 = q_values(i, 5);
    
    % Compute shoulder position
    shoulder_positions_calculated(i, :) = calculate_shoulder_position(q1, q21, q31, q4, q5, ...
        femur_to_humeris, pelvis_to_femur, upper_arm_length, forearm_length, ...
        r_to_p_x, r_to_p_y, r_to_p_z);

    shoulder_positions_calculated_v2(i, :) = calculate_shoulder_position_v2(q1, q21, q31, q4, q5, ...
        femur_to_humeris, pelvis_to_femur, upper_arm_length, forearm_length, ...
        r_to_p_x, r_to_p_y, r_to_p_z);
end

% Create a 3D scatter plot comparing actual, calculated, and calculated_v2 shoulder positions
figure;
hold on;
grid on;
axis equal;

% Plot actual shoulder positions
scatter3(shoulder_positions_adjusted(:,1), ...
         shoulder_positions_adjusted(:,2), ...
         shoulder_positions_adjusted(:,3), ...
         50, 'k', 'filled', 'DisplayName', 'Actual Position');

% Plot calculated shoulder positions
scatter3(shoulder_positions_calculated(:,1), ...
         shoulder_positions_calculated(:,2), ...
         shoulder_positions_calculated(:,3), ...
         50, 'm', 'filled', 'DisplayName', 'Calculated Position (old)');

% Plot calculated shoulder positions v2
scatter3(shoulder_positions_calculated_v2(:,1), ...
         shoulder_positions_calculated_v2(:,2), ...
         shoulder_positions_calculated_v2(:,3), ...
         50, 'y', 'filled', 'DisplayName', 'Calculated Position (corrected)');

% Labels and title
xlabel('X (meters)');
ylabel('Y (meters)');
zlabel('Z (meters)');
title('Comparison of Shoulder Position Estimations');

% Add legend
legend('Location', 'best');

% Adjust view
view(3);
hold off;

% Create a figure with subplots for X, Y, and Z comparisons
figure;

% Plot X-coordinates
subplot(3,1,1);
hold on; grid on;
scatter(1:num_positions, shoulder_positions_adjusted(:,1), 50, 'k', 'filled', 'DisplayName', 'Actual X');
scatter(1:num_positions, shoulder_positions_calculated(:,1), 50, 'm', 'DisplayName', 'Calculated X (old)');
scatter(1:num_positions, shoulder_positions_calculated_v2(:,1), 50, 'g^', 'DisplayName', 'Calculated X (corrected)');
ylabel('X (meters)');
title('X Coordinate Comparison');
legend('Location', 'best');

% Plot Y-coordinates
subplot(3,1,2);
hold on; grid on;
scatter(1:num_positions, shoulder_positions_adjusted(:,2), 50, 'k', 'filled', 'DisplayName', 'Actual Y');
scatter(1:num_positions, shoulder_positions_calculated(:,2), 50, 'm', 'filled', 'filled', 'DisplayName', 'Calculated Y (old)');
scatter(1:num_positions, shoulder_positions_calculated_v2(:,2), 50, 'y', 'filled', 'filled', 'DisplayName', 'Calculated Y (corrected)');
ylabel('Y (meters)');
title('Y Coordinate Comparison');
legend('Location', 'best');

% Plot Z-coordinates
subplot(3,1,3);
hold on; grid on;
scatter(1:num_positions, shoulder_positions_adjusted(:,3), 50, 'k', 'filled', 'DisplayName', 'Actual Z');
scatter(1:num_positions, shoulder_positions_calculated(:,3), 50, 'm', 'filled', 'DisplayName', 'Calculated Z (old)');
scatter(1:num_positions, shoulder_positions_calculated_v2(:,3), 50, 'y', 'filled', 'DisplayName', 'Calculated Z (corrected)');
ylabel('Z (meters)');
xlabel('Position Index');
title('Z Coordinate Comparison');
legend('Location', 'best');

% Adjust figure
sgtitle('Comparison of Shoulder Position Coordinates (X, Y, Z)');

%% Compute Error and Plot Average Error Comparison
% Compute errors for each coordinate component
error_x_old = abs(shoulder_positions_adjusted(:,1) - shoulder_positions_calculated(:,1));
error_y_old = abs(shoulder_positions_adjusted(:,2) - shoulder_positions_calculated(:,2));
error_z_old = abs(shoulder_positions_adjusted(:,3) - shoulder_positions_calculated(:,3));

error_x_new = abs(shoulder_positions_adjusted(:,1) - shoulder_positions_calculated_v2(:,1));
error_y_new = abs(shoulder_positions_adjusted(:,2) - shoulder_positions_calculated_v2(:,2));
error_z_new = abs(shoulder_positions_adjusted(:,3) - shoulder_positions_calculated_v2(:,3));

% Compute mean absolute errors
mean_error_old = [mean(error_x_old), mean(error_y_old), mean(error_z_old)];
mean_error_new = [mean(error_x_new), mean(error_y_new), mean(error_z_new)];

% Plot bar chart for average error comparison
figure;
bar([mean_error_old; mean_error_new]);

% Customize plot
xticklabels({'Old Model', 'Corrected Model'});
ylabel('Average Error (meters)');
title('Comparison of Average Shoulder Position Errors');
legend({'X', 'Y', 'Z'}, 'Location', 'best');
grid on;

%% Function Declarations
function Pp_s = calculate_shoulder_position(q1, q21, q31, q4, q5, T_L, T_W, U_L, F_L, r_to_p_x, r_to_p_y, r_to_p_z)
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

    %-----------------------------------------------------------------------
    % Changed from (Pp_e_x - T_W) to (Pp_e_x + T_W)
    % Added estimated_h2 and changed the calcualtion of cal_HS from 
    % Pp_e_z + Eproj_r to Pp_e_z + Eproj_r * cos(estimated_h2)
    %-----------------------------------------------------------------------
    Eproj_r = sqrt(U_L^2 - (Pp_e_x + T_W)^2); % projected radius of elbow position in sagittal plane
    l_H_Eproj = sqrt(Pp_e_y^2 + Pp_e_z^2); % distance between projected elbow joint and hip joint in sagittal plane
    estimated_h2 = atan2(-Pp_e_y,(T_L-Pp_e_z));
    cal_HS = Pp_e_z + Eproj_r * cos(estimated_h2);

    %-----------------------------------------------------------------------
    % Changed from Pp_e_y to -Pp_e_y
    % Changed from T_W to -T_W
    % Changed from yout(2) to -yout(2)
    % When cal_HS < T_L, changed amend_Eproj_r from T_L - Pp_e_z to
    % (T_L - Pp_e_z) / cos(estimated_h2)
    %-----------------------------------------------------------------------
    Eproj_r = sqrt(U_L^2 - (Pp_e_x + T_W)^2); % projected radius of elbow position in sagittal plane
    theta_elbow = atan(Pp_e_z/-Pp_e_y);
    L_hip_to_elbow = sqrt(Pp_e_y^2 + Pp_e_z^2);
    theta_elbow_to_torso = acos((T_L^2 + L_hip_to_elbow^2 - Eproj_r^2)/(2*T_L*L_hip_to_elbow));
    y_shoulder = -T_L*cos(theta_elbow - theta_elbow_to_torso);
    z_shoulder = T_L*sin(theta_elbow - theta_elbow_to_torso);
    Pp_s = [-T_W;y_shoulder;z_shoulder];
end

function Pp_s = calculate_shoulder_position_v2(q1, q21, q31, q4, q5, T_L, T_W, U_L, F_L, r_to_p_x, r_to_p_y, r_to_p_z)
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