%% Init
clc
clear
close all

%% Parameter initilization
% Robot base to pevlis
r_to_p_x = -0.385;
r_to_p_y = 0.166;
r_to_p_z = 0.0465;

% Model antrhopomettric data
upper_arm_length = 0.2486838396567298;
forearm_length = 0.25543492290745995;
femur_to_humeris = 0.355;
pelvis_to_femur = 0.163;

%% Read data
results_file = 'C:/Users/alexl/Desktop/ARAE_Calibration/MuJoCo Simulation/results/fixed/results.csv';
json_filename = 'C:/Users/alexl/Desktop/ARAE_Calibration/MuJoCo Simulation/simulation_data.json';

opts = detectImportOptions(results_file);
opts.DataLines = [2 Inf];

% Read the data from the file
data = readtable(results_file, opts);

positions = unique(data{:, 1});
num_positions = numel(positions);

% Preallocate cell arrays to store q_values and torques
q_values = cell(num_positions);
torques = cell(num_positions);
theoretical_torques = cell(num_positions);
theoretical_torques_v2 = cell(num_positions);

for i = 1:num_positions
    position_data = data(i, :);
    q_values{i} = [position_data{:, 2:6}];
    torques{i} = [position_data{:, 7:9}];
end

T_L = femur_to_humeris;
T_W = pelvis_to_femur;
U_L = upper_arm_length;
F_L = forearm_length;
g = 9.81;
F_M = 0.93874053;
Lgf = 0.4974308358852518;
U_M = 1.0542687374296;
Ugf = 0.57370836881454366;

for i = 1:num_positions
    
    q1 = q_values{i}(1);
    q21 = q_values{i}(2);
    q31 = q_values{i}(3);
    q4 = q_values{i}(4);
    q5 = q_values{i}(5);
    
    theoretical_torques{i} = compute_joint_torques(q1, q21, q31, q4, q5, T_L, T_W, U_L, F_L, g, Ugf*U_M, Lgf, F_M, r_to_p_x, r_to_p_y, r_to_p_z);
    theoretical_torques_v2{i} = compute_joint_torques_v2(q1, q21, q31, q4, q5, T_L, T_W, U_L, F_L, g, Lgf, F_M, Ugf, U_M, r_to_p_x, r_to_p_y, r_to_p_z);
end

%% Plot

% Convert cell arrays to numeric matrices
num_positions = length(theoretical_torques);
num_joints = 3; % Assuming torques have 3 components (for shoulder, elbow, etc.)

% Initialize matrices
percentage_error_1 = zeros(num_positions, num_joints);
percentage_error_2 = zeros(num_positions, num_joints);
actual_torques_matrix = zeros(num_positions, num_joints);
theoretical_torques_matrix = zeros(num_positions, num_joints);
theoretical_torques_v2_matrix = zeros(num_positions, num_joints);

for i = 1:num_positions
    % Convert cell entries to numeric arrays
    theor_torque = theoretical_torques{i}; % 1x3 vector
    theor_torque_v2 = theoretical_torques_v2{i}; % 1x3 vector
    actual_torque = torques{i}; % 1x3 vector
    
    % Store values in matrices for plotting
    theoretical_torques_matrix(i, :) = theor_torque;
    theoretical_torques_v2_matrix(i, :) = theor_torque_v2;
    actual_torques_matrix(i, :) = actual_torque;
    
    % Compute percentage error (element-wise)
    percentage_error_1(i, :) = abs(theor_torque - actual_torque) ./ abs(actual_torque) * 100;
    percentage_error_2(i, :) = abs(theor_torque_v2 - actual_torque) ./ abs(actual_torque) * 100;
end

% Plot theoretical vs actual torques for each joint
figure;
for j = 1:num_joints
    subplot(3,1,j); % Create subplots for each joint torque
    hold on;
    plot(1:num_positions, theoretical_torques_matrix(:, j), '-o', 'LineWidth', 1.5, 'DisplayName', ['Theoretical Joint ' num2str(j)]);
    plot(1:num_positions, theoretical_torques_v2_matrix(:, j), '-^', 'LineWidth', 1.5, 'DisplayName', ['Theoretical V2 Joint ' num2str(j)]);
    plot(1:num_positions, actual_torques_matrix(:, j), '-s', 'LineWidth', 1.5, 'DisplayName', ['Actual Joint ' num2str(j)]);
    xlabel('Position Index');
    ylabel('Torque (Nm)');
    title(['Joint ', num2str(j), ' Torques']);
    legend;
    grid on;
    hold off;
end

% Plot percentage error
figure;
for j = 1:num_joints
    subplot(3,1,j); % Create subplots for each joint torque error
    hold on;
    plot(1:num_positions, percentage_error_1(:, j), '-o', 'LineWidth', 1.5, 'DisplayName', 'Error (Theoretical vs Actual)');
    plot(1:num_positions, percentage_error_2(:, j), '-s', 'LineWidth', 1.5, 'DisplayName', 'Error (Theoretical V2 vs Actual)');
    xlabel('Position Index');
    ylabel('Percentage Error (%)');
    title(['Joint ', num2str(j), ' Percentage Error']);
    legend;
    grid on;
    hold off;
end

function T_hd = compute_joint_torques_v2(q1, q21, q31, q4, q5, T_L, T_W, U_L, F_L, g, Lgf, F_M, Ugf, U_M, r_to_p_x, r_to_p_y, r_to_p_z)
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

    % disp("Elbow in robot frame")
    % disp(P0_e)
    % 
    % disp("Elbow in pelvis frame")
    % disp(Pp_e)

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
    if cal_HS >= T_L
        amend_Eproj_r = Eproj_r;
        [yout,zout] = circcirc(0,0,T_L,-Pp_e_y,Pp_e_z,amend_Eproj_r);
        Pp_s = [-T_W;-yout(1);zout(1)];
    else
        % disp("Projected length < required length")
        amend_Eproj_r = (T_L - Pp_e_z) / cos(estimated_h2);
        [yout,zout] = circcirc(0,0,T_L,-Pp_e_y,Pp_e_z,amend_Eproj_r);
        Pp_s = [-T_W;-yout(1);zout(1)];
    end
    
    % disp("Shoulder in pelvis frame")
    % disp(Pp_s)

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
    
    % disp("Elbow position in shoulder frame")
    % disp(P_e)
    
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

    % disp("Human joint angles")
    % disp(h1)
    % disp(h2)
    % disp(h3)
    % disp(h4)
    
    % Applying human arm dynamic model to find support force
    Lr_f = F_L/2;
    
    if h4 > 80/180*pi
        h4 = 80/180*pi;
    elseif h4 < -80/180*pi
        h4 = -80/180*pi;
    end
    
    E_FCOM = F_L * Lgf;
    S_FCOM_J = [U_L*cos(h1)*cos(h2) - E_FCOM*cos(h4)*(cos(h3)*sin(h1) + cos(h1)*sin(h2)*sin(h3)) - E_FCOM*cos(h1)*cos(h2)*sin(h4), E_FCOM*sin(h1)*sin(h2)*sin(h4) - U_L*sin(h1)*sin(h2) - E_FCOM*cos(h2)*cos(h4)*sin(h1)*sin(h3), -E_FCOM*cos(h4)*(cos(h1)*sin(h3) + cos(h3)*sin(h1)*sin(h2)), -E_FCOM*sin(h4)*(cos(h1)*cos(h3) - sin(h1)*sin(h2)*sin(h3)) - E_FCOM*cos(h2)*cos(h4)*sin(h1); U_L*cos(h2)*sin(h1) + E_FCOM*cos(h4)*(cos(h1)*cos(h3) - sin(h1)*sin(h2)*sin(h3)) - E_FCOM*cos(h2)*sin(h1)*sin(h4), U_L*cos(h1)*sin(h2) - E_FCOM*cos(h1)*sin(h2)*sin(h4) + E_FCOM*cos(h1)*cos(h2)*cos(h4)*sin(h3), -E_FCOM*cos(h4)*(sin(h1)*sin(h3) - cos(h1)*cos(h3)*sin(h2)), E_FCOM*cos(h1)*cos(h2)*cos(h4) - E_FCOM*sin(h4)*(cos(h3)*sin(h1) + cos(h1)*sin(h2)*sin(h3)); 0, E_FCOM*cos(h2)*sin(h4) - U_L*cos(h2) + E_FCOM*cos(h4)*sin(h2)*sin(h3), -E_FCOM*cos(h2)*cos(h3)*cos(h4), E_FCOM*cos(h4)*sin(h2) + E_FCOM*cos(h2)*sin(h3)*sin(h4)];
    S_FCOM_J_T = S_FCOM_J.';
    F_FCOM = [0;0;-F_M*g];
    S_FCOM_tau = S_FCOM_J_T*F_FCOM;
    
    S_UCOM = U_L * Ugf;
    S_UCOM_J = [S_UCOM*cos(h1)*cos(h2), -S_UCOM*sin(h1)*sin(h2), 0; S_UCOM*sin(h1)*cos(h2), S_UCOM*cos(h1)*sin(h2), 0; 0, -S_UCOM*cos(h2), 0];
    S_UCOM_J_T = S_UCOM_J.';
    F_UCOM = [0;0;-U_M*g];
    S_UCOM_tau = S_UCOM_J_T*F_UCOM;

    t1g = S_FCOM_tau(1) + S_UCOM_tau(1);
    t2g = S_FCOM_tau(2) + S_UCOM_tau(2);
    t3g = S_FCOM_tau(3) + S_UCOM_tau(3);
    t4g = S_FCOM_tau(4);

    disp(S_FCOM_tau)
    disp(S_UCOM_tau)

    % t1g = 0;
    % t2g = (-F_M*g*(F_L*Lgf*(sin(h1)*cos(h3)*cos(h4)+cos(h1)*sin(h2)*sin(h3)*cos(h4)+cos(h1)*cos(h2)*sin(h4))-U_L*cos(h1)*cos(h2))+Ugf_U_M*g*U_L*cos(h1)*cos(h2))*cos(h1) + (F_M*g*(F_L*Lgf*(cos(h1)*cos(h3)*cos(h4)-sin(h1)*sin(h2)*sin(h3)*cos(h4)-sin(h1)*cos(h2)*sin(h4))+U_L*sin(h1)*cos(h2))+Ugf_U_M*g*U_L*sin(h1)*cos(h2))*sin(h1);
    % t3g = (-F_M*g*(F_L*Lgf*(sin(h1)*cos(h3)*cos(h4)+cos(h1)*sin(h2)*sin(h3)*cos(h4)+cos(h1)*cos(h2)*sin(h4))-U_L*cos(h1)*cos(h2))+Ugf_U_M*g*U_L*cos(h1)*cos(h2))*(-sin(h1)*cos(h2)) + (F_M*g*(F_L*Lgf*(cos(h1)*cos(h3)*cos(h4)-sin(h1)*sin(h2)*sin(h3)*cos(h4)-sin(h1)*cos(h2)*sin(h4))+U_L*sin(h1)*cos(h2))+Ugf_U_M*g*U_L*sin(h1)*cos(h2))*(cos(h1)*cos(h2));
    % t4g = (-F_M*g*(F_L*Lgf*(sin(h1)*cos(h3)*cos(h4)+cos(h1)*sin(h2)*sin(h3)*cos(h4)+cos(h1)*cos(h2)*sin(h4))-U_L*cos(h1)*cos(h2))+Ugf_U_M*g*U_L*cos(h1)*cos(h2))*(cos(h1)*sin(h3)+sin(h1)*sin(h2)*cos(h3)) + (F_M*g*(F_L*Lgf*(cos(h1)*cos(h3)*cos(h4)-sin(h1)*sin(h2)*sin(h3)*cos(h4)-sin(h1)*cos(h2)*sin(h4))+U_L*sin(h1)*cos(h2))+Ugf_U_M*g*U_L*sin(h1)*cos(h2))*(sin(h1)*sin(h3)-cos(h1)*sin(h2)*cos(h3));
    
    % t1g = 0;
    % t2g = (F_M*(F_L*Lgf*cos(h2)*sin(h4) - U_L*cos(h2) + F_L*Lgf*cos(h4)*sin(h2)*sin(h3)) - U_L*Ugf_U_M*cos(h2))*g;
    % t3g = (-F_L*Lgf*F_M*cos(h2)*cos(h3)*cos(h4))*g;
    % t4g = (F_M*(F_L*Lgf*cos(h4)*sin(h2) + F_L*Lgf*cos(h2)*sin(h3)*sin(h4)))*g;

    T_s_r = [0 -1  0;
             1  0  0;
             0  0  1];

    J_h_2 = [U_L*cos(h1)*cos(h2) - Lr_f*cos(h4)*(cos(h3)*sin(h1) + cos(h1)*sin(h2)*sin(h3)) - Lr_f*cos(h1)*cos(h2)*sin(h4), Lr_f*sin(h1)*sin(h2)*sin(h4) - U_L*sin(h1)*sin(h2) - Lr_f*cos(h2)*cos(h4)*sin(h1)*sin(h3), -Lr_f*cos(h4)*(cos(h1)*sin(h3) + cos(h3)*sin(h1)*sin(h2)), -Lr_f*sin(h4)*(cos(h1)*cos(h3) - sin(h1)*sin(h2)*sin(h3)) - Lr_f*cos(h2)*cos(h4)*sin(h1); U_L*cos(h2)*sin(h1) + Lr_f*cos(h4)*(cos(h1)*cos(h3) - sin(h1)*sin(h2)*sin(h3)) - Lr_f*cos(h2)*sin(h1)*sin(h4), U_L*cos(h1)*sin(h2) - Lr_f*cos(h1)*sin(h2)*sin(h4) + Lr_f*cos(h1)*cos(h2)*cos(h4)*sin(h3), -Lr_f*cos(h4)*(sin(h1)*sin(h3) - cos(h1)*cos(h3)*sin(h2)), Lr_f*cos(h1)*cos(h2)*cos(h4) - Lr_f*sin(h4)*(cos(h3)*sin(h1) + cos(h1)*sin(h2)*sin(h3)); 0, Lr_f*cos(h2)*sin(h4) - U_L*cos(h2) + Lr_f*cos(h4)*sin(h2)*sin(h3), -Lr_f*cos(h2)*cos(h3)*cos(h4), Lr_f*cos(h4)*sin(h2) + Lr_f*cos(h2)*sin(h3)*sin(h4)];
    Tg_h_2 = [t1g;t2g;t3g;t4g];
    pv_J_h = pinv(J_h_2.');
    F_r2_s = pv_J_h*Tg_h_2;
    F_r2 = T_s_r * F_r2_s;
    
    % Find torque required by each motor
    
    %-----------------------------------------------------------
    % Changed from w14+F_r2(3) to -w14+F_r2(3)
    %-----------------------------------------------------------

    wl4 = (0.6774) * g;
    F = [-F_r2(1);
        -F_r2(2);
        wl4-F_r2(3)];
    J = [-l21*cos(q21 - pi/2)*sin(q1) - cos(q21 + q31 + pi/2)*cos(q21 - pi/2)*sin(q1)*(l22 - l31) - sin(q21 + q31 + pi/2)*sin(q1)*sin(q21 - pi/2)*(l22 - l31), -l21*cos(q1)*sin(q21 - pi/2), cos(q21 + q31 + pi/2)*cos(q1)*sin(q21 - pi/2)*(l22 - l31) - sin(q21 + q31 + pi/2)*cos(q1)*cos(q21 - pi/2)*(l22 - l31); l21*cos(q1)*cos(q21 - pi/2) + cos(q21 + q31 + pi/2)*cos(q1)*cos(q21 - pi/2)*(l22 - l31) + sin(q21 + q31 + pi/2)*cos(q1)*sin(q21 - pi/2)*(l22 - l31), -l21*sin(q1)*sin(q21 - pi/2), cos(q21 + q31 + pi/2)*sin(q1)*sin(q21 - pi/2)*(l22 - l31) - sin(q21 + q31 + pi/2)*cos(q21 - pi/2)*sin(q1)*(l22 - l31); 0, -l21*cos(q21 - pi/2), cos(q21 + q31 + pi/2)*cos(q21 - pi/2)*(l22 - l31) + sin(q21 + q31 + pi/2)*sin(q21 - pi/2)*(l22 - l31)];
    JF = (J.')*F;

    %-----------------------------------------------------------
    % Changed from JF(n,n) to -JF(n,n)
    %-----------------------------------------------------------
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

    % disp("Elbow in robot frame")
    % disp(P0_e)
    % 
    % disp("Elbow in pelvis frame")
    % disp(Pp_e)

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
    S_predicted = [0, T_L];
    if cal_HS >= T_L
        amend_Eproj_r = Eproj_r;
        [yout,zout] = circcirc(0,0,T_L,-Pp_e_y,Pp_e_z,amend_Eproj_r);
        S_calculated_1 = [yout(1),zout(1)];
        S_calculated_2 = [yout(2),zout(2)];
        if (norm(S_predicted- S_calculated_1) < norm(S_predicted- S_calculated_2))
            Pp_s = [-T_W;-yout(1);zout(1)];
            disp("First intersection")
        else 
            Pp_s = [-T_W;-yout(2);zout(2)];
            disp("Second intersection")
        end
    else
        disp("Projected length < required length")
        amend_Eproj_r = (T_L - Pp_e_z) / cos(estimated_h2);
        [yout,zout] = circcirc(0,0,T_L,-Pp_e_y,Pp_e_z,amend_Eproj_r);
        S_calculated_1 = [yout(1),zout(1)];
        S_calculated_2 = [yout(2),zout(2)];
        if (norm(S_predicted- S_calculated_1) < norm(S_predicted- S_calculated_2))
            Pp_s = [-T_W;-yout(1);zout(1)];
            disp("First intersection")
        else 
            Pp_s = [-T_W;-yout(2);zout(2)];
            disp("Second intersection")
        end
    end
    
    % disp("Shoulder in pelvis frame")
    % disp(Pp_s)

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
    
    disp("Elbow position in shoulder frame")
    disp(P_e)
    
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

    % disp("Human joint angles")
    % disp(h1)
    % disp(h2)
    % disp(h3)
    % disp(h4)
    
    % Applying human arm dynamic model to find support force
    Lr_f = F_L/2;
    
    if h4 > 80/180*pi
        h4 = 80/180*pi;
    elseif h4 < -80/180*pi
        h4 = -80/180*pi;
    end
    
    % E_FCOM = F_L * Lgf;
    % S_FCOM_J = [U_L*cos(h1)*cos(h2) - E_FCOM*cos(h4)*(cos(h3)*sin(h1) + cos(h1)*sin(h2)*sin(h3)) - E_FCOM*cos(h1)*cos(h2)*sin(h4), E_FCOM*sin(h1)*sin(h2)*sin(h4) - U_L*sin(h1)*sin(h2) - E_FCOM*cos(h2)*cos(h4)*sin(h1)*sin(h3), -E_FCOM*cos(h4)*(cos(h1)*sin(h3) + cos(h3)*sin(h1)*sin(h2)), -E_FCOM*sin(h4)*(cos(h1)*cos(h3) - sin(h1)*sin(h2)*sin(h3)) - E_FCOM*cos(h2)*cos(h4)*sin(h1); U_L*cos(h2)*sin(h1) + E_FCOM*cos(h4)*(cos(h1)*cos(h3) - sin(h1)*sin(h2)*sin(h3)) - E_FCOM*cos(h2)*sin(h1)*sin(h4), U_L*cos(h1)*sin(h2) - E_FCOM*cos(h1)*sin(h2)*sin(h4) + E_FCOM*cos(h1)*cos(h2)*cos(h4)*sin(h3), -E_FCOM*cos(h4)*(sin(h1)*sin(h3) - cos(h1)*cos(h3)*sin(h2)), E_FCOM*cos(h1)*cos(h2)*cos(h4) - E_FCOM*sin(h4)*(cos(h3)*sin(h1) + cos(h1)*sin(h2)*sin(h3)); 0, E_FCOM*cos(h2)*sin(h4) - U_L*cos(h2) + E_FCOM*cos(h4)*sin(h2)*sin(h3), -E_FCOM*cos(h2)*cos(h3)*cos(h4), E_FCOM*cos(h4)*sin(h2) + E_FCOM*cos(h2)*sin(h3)*sin(h4)];
    % S_FCOM_J_T = S_FCOM_J.';
    % F_FCOM = [0;0;-F_M*g];
    % S_FCOM_tau = S_FCOM_J_T*F_FCOM;
    % 
    % U_M = 1.0542687374296; % Dummy value
    % Ugf = 0.57370836881454366; % Dummy value
    % S_UCOM = U_L * Ugf;
    % S_UCOM_J = [S_UCOM*cos(h1)*cos(h2), -S_UCOM*sin(h1)*sin(h2), 0; S_UCOM*sin(h1)*cos(h2), S_UCOM*cos(h1)*sin(h2), 0; 0, -S_UCOM*cos(h2), 0];
    % S_UCOM_J_T = S_UCOM_J.';
    % F_UCOM = [0;0;-U_M*g];
    % S_UCOM_tau = S_UCOM_J_T*F_UCOM;
    % 
    % t1g = S_FCOM_tau(1) + S_UCOM_tau(1);
    % t2g = S_FCOM_tau(2) + S_UCOM_tau(2);
    % t3g = S_FCOM_tau(3) + S_UCOM_tau(3);
    % t4g = S_FCOM_tau(4);

    % t1g = 0;
    % t2g = (-F_M*g*(F_L*Lgf*(sin(h1)*cos(h3)*cos(h4)+cos(h1)*sin(h2)*sin(h3)*cos(h4)+cos(h1)*cos(h2)*sin(h4))-U_L*cos(h1)*cos(h2))+Ugf_U_M*g*U_L*cos(h1)*cos(h2))*cos(h1) + (F_M*g*(F_L*Lgf*(cos(h1)*cos(h3)*cos(h4)-sin(h1)*sin(h2)*sin(h3)*cos(h4)-sin(h1)*cos(h2)*sin(h4))+U_L*sin(h1)*cos(h2))+Ugf_U_M*g*U_L*sin(h1)*cos(h2))*sin(h1);
    % t3g = (-F_M*g*(F_L*Lgf*(sin(h1)*cos(h3)*cos(h4)+cos(h1)*sin(h2)*sin(h3)*cos(h4)+cos(h1)*cos(h2)*sin(h4))-U_L*cos(h1)*cos(h2))+Ugf_U_M*g*U_L*cos(h1)*cos(h2))*(-sin(h1)*cos(h2)) + (F_M*g*(F_L*Lgf*(cos(h1)*cos(h3)*cos(h4)-sin(h1)*sin(h2)*sin(h3)*cos(h4)-sin(h1)*cos(h2)*sin(h4))+U_L*sin(h1)*cos(h2))+Ugf_U_M*g*U_L*sin(h1)*cos(h2))*(cos(h1)*cos(h2));
    % t4g = (-F_M*g*(F_L*Lgf*(sin(h1)*cos(h3)*cos(h4)+cos(h1)*sin(h2)*sin(h3)*cos(h4)+cos(h1)*cos(h2)*sin(h4))-U_L*cos(h1)*cos(h2))+Ugf_U_M*g*U_L*cos(h1)*cos(h2))*(cos(h1)*sin(h3)+sin(h1)*sin(h2)*cos(h3)) + (F_M*g*(F_L*Lgf*(cos(h1)*cos(h3)*cos(h4)-sin(h1)*sin(h2)*sin(h3)*cos(h4)-sin(h1)*cos(h2)*sin(h4))+U_L*sin(h1)*cos(h2))+Ugf_U_M*g*U_L*sin(h1)*cos(h2))*(sin(h1)*sin(h3)-cos(h1)*sin(h2)*cos(h3));
    
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
    
    % Find torque required by each motor
    
    %-----------------------------------------------------------
    % Changed from w14+F_r2(3) to -w14+F_r2(3)
    %-----------------------------------------------------------

    wl4 = (0.6774) * g;
    F = [F_r2(1);
        F_r2(2);
        wl4+F_r2(3)];
    J = [-l21*cos(q21 - pi/2)*sin(q1) - cos(q21 + q31 + pi/2)*cos(q21 - pi/2)*sin(q1)*(l22 - l31) - sin(q21 + q31 + pi/2)*sin(q1)*sin(q21 - pi/2)*(l22 - l31), -l21*cos(q1)*sin(q21 - pi/2), cos(q21 + q31 + pi/2)*cos(q1)*sin(q21 - pi/2)*(l22 - l31) - sin(q21 + q31 + pi/2)*cos(q1)*cos(q21 - pi/2)*(l22 - l31); l21*cos(q1)*cos(q21 - pi/2) + cos(q21 + q31 + pi/2)*cos(q1)*cos(q21 - pi/2)*(l22 - l31) + sin(q21 + q31 + pi/2)*cos(q1)*sin(q21 - pi/2)*(l22 - l31), -l21*sin(q1)*sin(q21 - pi/2), cos(q21 + q31 + pi/2)*sin(q1)*sin(q21 - pi/2)*(l22 - l31) - sin(q21 + q31 + pi/2)*cos(q21 - pi/2)*sin(q1)*(l22 - l31); 0, -l21*cos(q21 - pi/2), cos(q21 + q31 + pi/2)*cos(q21 - pi/2)*(l22 - l31) + sin(q21 + q31 + pi/2)*sin(q21 - pi/2)*(l22 - l31)];
    JF = (J.')*F;

    %-----------------------------------------------------------
    % Changed from JF(n,n) to -JF(n,n)
    %-----------------------------------------------------------
    t1g  = JF(1,1);
    t21g = JF(2,1) + (- m22*(l32*sin(q21 + q31 - (5*pi)/2)*sin(q31) + l32*cos(q21 + q31 - (5*pi)/2)*cos(q31)) - m32*(x32*cos(q21 + q31 - (5*pi)/2)*cos(q31) + x32*sin(q21 + q31 - (5*pi)/2)*sin(q31)) - lg21*m21*cos(r21 - q21 + pi/2))*g;
    t31g = JF(3,1) + (m22*(l31*cos(q31) + lg22*cos(q21 + q31 + r22 + pi/2)*(cos(q21 + q31 - (5*pi)/2)*cos(q31) + sin(q21 + q31 - (5*pi)/2)*sin(q31)) - lg22*sin(q21 + q31 + r22 + pi/2)*(cos(q21 + q31 - (5*pi)/2)*sin(q31) - sin(q21 + q31 - (5*pi)/2)*cos(q31))) + l31*m32*cos(q31) + m31*x31*cos(q31))*g;
    
    T_hd = [t1g,t21g,t31g];
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