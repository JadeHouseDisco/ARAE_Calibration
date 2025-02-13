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
results_file = 'C:/Users/alexl/Desktop/ARAE_Calibration/MuJoCo Simulation/results/robot_only_20p/results.csv';
json_filename = 'C:/Users/alexl/Desktop/ARAE_Calibration/MuJoCo Simulation/simulation_data.json';

opts = detectImportOptions(results_file);
opts.DataLines = [2 Inf];

% Read the data from the file
data = readtable(results_file, opts);

positions = unique(data{:, 1});
num_positions = numel(positions);

% Preallocate cell arrays to store q_values and torques
q_values = cell(num_positions);
simulation_torques = cell(num_positions);

for i = 1:num_positions
    position_data = data(i, :);
    q_values{i} = position_data{:, 2:6};
    simulation_torques{i} = position_data{:, 7:9}; 
end

%% Solver initilization
g = 9.81;

theoretical_torque = cell(num_positions);

for i = 1:num_positions
    theoretical_torque{i} = compute_joint_torques(q_values{i}(1), q_values{i}(2), q_values{i}(3), q_values{i}(4), q_values{i}(5), g);
end

%% Compare torques and calculate errors
errors = cell(num_positions, 1); % Preallocate cell array for errors

for i = 1:num_positions
    % Compute the absolute error between theoretical and simulation torques
    errors{i} = abs(theoretical_torque{i} - simulation_torques{i});
end

%% Separate errors by joints
joint_errors = zeros(num_positions, 3); % Preallocate matrix for errors (rows: positions, cols: joints)

for i = 1:num_positions
    % Extract errors for each joint
    joint_errors(i, :) = errors{i}'; % Transpose to align matrix dimensions
end

%% Plot errors for each joint separately
joint_labels = {'Joint 1', 'Joint 2', 'Joint 3'}; % Labels for joints

for j = 1:3
    figure; % Create a new figure for each joint
    plot(1:num_positions, joint_errors(:, j), 'o-'); % Plot errors for the current joint
    xlabel('Position Index');
    ylabel('Torque Error');
    title(['Error for ', joint_labels{j}]);
    grid on;
end

%% Compare torques and calculate percentage errors for joints 2 and 3
percentage_errors = zeros(num_positions, 3); % Preallocate matrix for percentage errors (rows: positions, cols: joints)

for i = 1:num_positions
    % Compute percentage error for joint 2 and joint 3
    for j = 2:3
        theoretical = abs(theoretical_torque{i}(j));
        simulation = abs(simulation_torques{i}(j));
        percentage_errors(i, j) = abs(theoretical - simulation) / max(simulation, eps) * 100; % Avoid division by zero
    end
end

%% Plot percentage errors for joint 2 and joint 3
joint_labels = {'Joint 2', 'Joint 3'}; % Labels for joints

for j = 2:3
    figure; % Create a new figure for each joint
    plot(1:num_positions, percentage_errors(:, j), 'o-'); % Plot percentage errors for the current joint
    xlabel('Position Index');
    ylabel('Percentage Error (%)');
    title(['Percentage Error for ', joint_labels{j}]);
    grid on;
end



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

function T_hd = compute_joint_torques(q1, q21, q31, q4, q5, g)
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
    
    % t1g  = JF(1,1);
    % t21g = JF(2,1) + (- m22*(l32*sin(q21 + q31 - (5*pi)/2)*sin(q31) + l32*cos(q21 + q31 - (5*pi)/2)*cos(q31)) - m32*(x32*cos(q21 + q31 - (5*pi)/2)*cos(q31) + x32*sin(q21 + q31 - (5*pi)/2)*sin(q31)) - lg21*m21*cos(r21 - q21 + pi/2))*g;
    % t31g = JF(3,1) + (m22*(l31*cos(q31) + lg22*cos(q21 + q31 + r22 + pi/2)*(cos(q21 + q31 - (5*pi)/2)*cos(q31) + sin(q21 + q31 - (5*pi)/2)*sin(q31)) - lg22*sin(q21 + q31 + r22 + pi/2)*(cos(q21 + q31 - (5*pi)/2)*sin(q31) - sin(q21 + q31 - (5*pi)/2)*cos(q31))) + l31*m32*cos(q31) + m31*x31*cos(q31))*g;
    
    t1g  = 0;
    t21g = (- m22*(l32*sin(q21 + q31 - (5*pi)/2)*sin(q31) + l32*cos(q21 + q31 - (5*pi)/2)*cos(q31)) - m32*(x32*cos(q21 + q31 - (5*pi)/2)*cos(q31) + x32*sin(q21 + q31 - (5*pi)/2)*sin(q31)) - lg21*m21*cos(r21 - q21 + pi/2))*g;
    t31g = (m22*(l31*cos(q31) + lg22*cos(q21 + q31 + r22 + pi/2)*(cos(q21 + q31 - (5*pi)/2)*cos(q31) + sin(q21 + q31 - (5*pi)/2)*sin(q31)) - lg22*sin(q21 + q31 + r22 + pi/2)*(cos(q21 + q31 - (5*pi)/2)*sin(q31) - sin(q21 + q31 - (5*pi)/2)*cos(q31))) + l31*m32*cos(q31) + m31*x31*cos(q31))*g;

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