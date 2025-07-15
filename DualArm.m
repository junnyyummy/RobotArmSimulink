% Dual-arm inverse kinematics numerical solver and error analysis program
% Read end-effector positions from CSV file, solve inverse kinematics, and analyze joint angle and position errors

clc;
clear;
close all;

% Set CSV file path
logFilePath = 'joint_log.csv';

% Check if CSV file exists
if ~isfile(logFilePath)
    error('CSV file does not exist, please ensure joint_log.csv is in the working directory');
end

% Read CSV file
try
    data = readtable(logFilePath);
    fprintf('Successfully read CSV file, total %d rows\n', height(data));
catch ME
    error('Failed to read CSV file: %s', ME.message);
end

% Get number of columns to ensure sufficient data
if width(data) < 20
    error('CSV file has insufficient columns, at least 20 columns required');
end

% Extract data
% Arm 1 data
arm1_actual_joint_angles_rad = table2array(data(:, 1:4));    % Actual joint angles (radians)
arm1_target_positions_m = table2array(data(:, 9:11));        % Target end-effector positions (x,y,z, in meters)

% Arm 2 data
arm2_actual_joint_angles_rad = table2array(data(:, 5:8));    % Actual joint angles (radians)
arm2_target_positions_m = table2array(data(:, 15:17));       % Target end-effector positions (x,y,z, in meters)

% DH parameter table for Arm 1
DH_params_arm1 = [
    0,    109,   5.1,  -90;  % Transform from world frame to joint 1 (fixed)
    0,    5.3,   30.9,  90;  % Transform from joint 1 to joint 2
    0,    0,     270,  -90;  % Transform from joint 2 to joint 3
    90,   22.8,  0,     90;  % Transform from joint 3 to joint 4
    0,    250,   0,     0    % Transform from joint 4 to joint 5
];

% DH parameter table for Arm 2
DH_params_arm2 = [
    0,    109,   354.9,  90;  % Transform from world frame to joint 1 (fixed)
    0,    -128.8, 30.9, -90;  % Transform from joint 1 to joint 2
    0,    0,      270,   90;  % Transform from joint 2 to joint 3
    90,   22.8,      0,     90;  % Transform from joint 3 to joint 4
    0,    265,    0,     0    % Transform from joint 4 to joint 5
];

% Initialize result storage
num_points = size(arm1_target_positions_m, 1);

% Arm 1 result storage
arm1_solved_joint_angles_rad = zeros(num_points, 4);
arm1_joint_angle_errors_rad = zeros(num_points, 4);
arm1_joint_angle_errors_deg = zeros(num_points, 4);
arm1_calculated_positions_m = zeros(num_points, 3);
arm1_position_errors_m = zeros(num_points, 3);
arm1_convergence_flags = zeros(num_points, 1);

% Arm 2 result storage
arm2_solved_joint_angles_rad = zeros(num_points, 4);
arm2_joint_angle_errors_rad = zeros(num_points, 4);
arm2_joint_angle_errors_deg = zeros(num_points, 4);
arm2_calculated_positions_m = zeros(num_points, 3);
arm2_position_errors_m = zeros(num_points, 3);
arm2_convergence_flags = zeros(num_points, 1);

% Inverse kinematics solving parameters
options = optimset('Display', 'off', 'TolFun', 1e-6, 'TolX', 1e-6, 'MaxIter', 500);

fprintf('Starting dual-arm inverse kinematics solving...\n');

% Solve inverse kinematics for each target position
for i = 1:num_points
    % === Arm 1 solving ===
    target_pos_mm_arm1 = arm1_target_positions_m(i, :) * 1000;
    initial_guess_arm1 = arm1_actual_joint_angles_rad(i, :);

    objective_function_arm1 = @(joint_angles) inverse_kinematics_objective(joint_angles, target_pos_mm_arm1, DH_params_arm1);

    try
        [solved_angles_arm1, fval_arm1, exitflag_arm1] = fminsearch(objective_function_arm1, initial_guess_arm1, options);
        arm1_solved_joint_angles_rad(i, :) = solved_angles_arm1;
        arm1_convergence_flags(i) = (exitflag_arm1 == 1 && fval_arm1 < 1e-3);
    catch ME
        fprintf('Arm 1 data point %d failed to solve: %s\n', i, ME.message);
        arm1_solved_joint_angles_rad(i, :) = initial_guess_arm1;
        arm1_convergence_flags(i) = 0;
    end

    % Compute joint angle error for Arm 1
    arm1_joint_angle_errors_rad(i, :) = arm1_solved_joint_angles_rad(i, :) - arm1_actual_joint_angles_rad(i, :);
    arm1_joint_angle_errors_deg(i, :) = arm1_joint_angle_errors_rad(i, :) * 180 / pi;

    % Forward kinematics verification for Arm 1
    T_final_arm1 = forward_kinematics(arm1_solved_joint_angles_rad(i, :), DH_params_arm1);
    calculated_pos_mm_arm1 = T_final_arm1(1:3, 4);
    arm1_calculated_positions_m(i, :) = calculated_pos_mm_arm1' / 1000;
    arm1_position_errors_m(i, :) = arm1_calculated_positions_m(i, :) - arm1_target_positions_m(i, :);

    % === Arm 2 solving ===
    target_pos_mm_arm2 = arm2_target_positions_m(i, :) * 1000;
    initial_guess_arm2 = arm2_actual_joint_angles_rad(i, :);

    objective_function_arm2 = @(joint_angles) inverse_kinematics_objective(joint_angles, target_pos_mm_arm2, DH_params_arm2);

    try
        [solved_angles_arm2, fval_arm2, exitflag_arm2] = fminsearch(objective_function_arm2, initial_guess_arm2, options);
        arm2_solved_joint_angles_rad(i, :) = solved_angles_arm2;
        arm2_convergence_flags(i) = (exitflag_arm2 == 1 && fval_arm2 < 1e-3);
    catch ME
        fprintf('Arm 2 data point %d failed to solve: %s\n', i, ME.message);
        arm2_solved_joint_angles_rad(i, :) = initial_guess_arm2;
        arm2_convergence_flags(i) = 0;
    end

    % Compute joint angle error for Arm 2
    arm2_joint_angle_errors_rad(i, :) = arm2_solved_joint_angles_rad(i, :) - arm2_actual_joint_angles_rad(i, :);
    arm2_joint_angle_errors_deg(i, :) = arm2_joint_angle_errors_rad(i, :) * 180 / pi;

    % Forward kinematics verification for Arm 2
    T_final_arm2 = forward_kinematics(arm2_solved_joint_angles_rad(i, :), DH_params_arm2);
    calculated_pos_mm_arm2 = T_final_arm2(1:3, 4);
    arm2_calculated_positions_m(i, :) = calculated_pos_mm_arm2' / 1000;
    arm2_position_errors_m(i, :) = arm2_calculated_positions_m(i, :) - arm2_target_positions_m(i, :);

    % Display progress
    if mod(i, 100) == 0 || i == num_points
        fprintf('Processed %d/%d data points\n', i, num_points);
    end
end

% === Statistical results ===
% Arm 1 statistics
arm1_converged_count = sum(arm1_convergence_flags);
arm1_total_pos_error = sqrt(sum(arm1_position_errors_m.^2, 2));

% Arm 2 statistics
arm2_converged_count = sum(arm2_convergence_flags);
arm2_total_pos_error = sqrt(sum(arm2_position_errors_m.^2, 2));

% Display statistical results
print_statistics('Arm 1', arm1_converged_count, num_points, arm1_joint_angle_errors_rad, ...
                 arm1_joint_angle_errors_deg, arm1_position_errors_m, arm1_total_pos_error);

print_statistics('Arm 2', arm2_converged_count, num_points, arm2_joint_angle_errors_rad, ...
                 arm2_joint_angle_errors_deg, arm2_position_errors_m, arm2_total_pos_error);

% Save results to CSV file
save_results_to_csv(arm1_actual_joint_angles_rad, arm1_solved_joint_angles_rad, ...
                   arm1_joint_angle_errors_rad, arm1_joint_angle_errors_deg, ...
                   arm1_target_positions_m, arm1_calculated_positions_m, ...
                   arm1_position_errors_m, arm1_total_pos_error, arm1_convergence_flags, ...
                   arm2_actual_joint_angles_rad, arm2_solved_joint_angles_rad, ...
                   arm2_joint_angle_errors_rad, arm2_joint_angle_errors_deg, ...
                   arm2_target_positions_m, arm2_calculated_positions_m, ...
                   arm2_position_errors_m, arm2_total_pos_error, arm2_convergence_flags);

% Plot error charts
create_dual_arm_plots(arm1_joint_angle_errors_deg, arm1_position_errors_m, arm1_total_pos_error, arm1_convergence_flags, ...
                     arm2_joint_angle_errors_deg, arm2_position_errors_m, arm2_total_pos_error, arm2_convergence_flags);

fprintf('\nDual-arm inverse kinematics analysis program completed!\n');

Ts = 1; % Example sampling time
time_vector = (0:num_points-1)' * Ts;

% Combine timestamped data matrix (time column + 4 joint columns)
arm1_data = [time_vector, arm1_solved_joint_angles_rad];
arm2_data = [time_vector, arm2_solved_joint_angles_rad];

%% Inverse Kinematics Objective Function
function error = inverse_kinematics_objective(joint_angles, target_position_mm, DH_params)
    % Objective function: minimize end-effector position error
    T_final = forward_kinematics(joint_angles, DH_params);
    calculated_position_mm = T_final(1:3, 4);
    position_error = calculated_position_mm' - target_position_mm;
    error = sum(position_error.^2);
end

%% Forward Kinematics Calculation Function
function T_final = forward_kinematics(joint_angles_rad, DH_params_initial)
    % Forward kinematics calculation
    T_final = eye(4);
    
    for i = 1:size(DH_params_initial, 1)
        if i == 1
            theta_rad = DH_params_initial(i, 1) * pi / 180;
        else
            theta_initial_rad = DH_params_initial(i, 1) * pi / 180;
            joint_index = i - 1;
            
            if joint_index <= length(joint_angles_rad)
                theta_rad = theta_initial_rad + joint_angles_rad(joint_index);
            else
                theta_rad = theta_initial_rad;
            end
        end
        
        d_mm = DH_params_initial(i, 2);
        a_mm = DH_params_initial(i, 3);
        alpha_rad = DH_params_initial(i, 4) * pi / 180;
        
        T_i = dh_transform(theta_rad, d_mm, a_mm, alpha_rad);
        T_final = T_final * T_i;
    end
end

%% DH Transformation Matrix Calculation Function
function T = dh_transform(theta, d, a, alpha)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end

%% Print Statistics Function
function print_statistics(arm_name, converged_count, num_points, joint_angle_errors_rad, ...
                         joint_angle_errors_deg, position_errors_m, total_pos_error)
    fprintf('\n=== %s Inverse Kinematics Solved ===\n', arm_name);
    fprintf('Number of data points processed: %d\n', num_points);
    fprintf('Number of converged points: %d (%.1f%%)\n', converged_count, 100*converged_count/num_points);
    
    % Joint angle error statistics (radian)
    fprintf('\n=== %s Joint Angle Error Statistics (radian) ===\n', arm_name);
    joint_mean_error_rad = mean(abs(joint_angle_errors_rad), 1);
    joint_max_error_rad = max(abs(joint_angle_errors_rad), [], 1);
    joint_rms_error_rad = sqrt(mean(joint_angle_errors_rad.^2, 1));
    
    for j = 1:4
        fprintf('Joint %d - Mean Error: %.6f, Max Error: %.6f, RMS Error: %.6f\n', ...
                j, joint_mean_error_rad(j), joint_max_error_rad(j), joint_rms_error_rad(j));
    end
    
    % Joint angle error statistics (degree)
    fprintf('\n=== %s Joint Angle Error Statistics (degree) ===\n', arm_name);
    joint_mean_error_deg = mean(abs(joint_angle_errors_deg), 1);
    joint_max_error_deg = max(abs(joint_angle_errors_deg), [], 1);
    joint_rms_error_deg = sqrt(mean(joint_angle_errors_deg.^2, 1));
    
    for j = 1:4
        fprintf('Joint %d - Mean Error: %.3f, Max Error: %.3f, RMS Error: %.3f\n', ...
                j, joint_mean_error_deg(j), joint_max_error_deg(j), joint_rms_error_deg(j));
    end
    
    % Position error statistics
    fprintf('\n=== %s Position Error Statistics (m) ===\n', arm_name);
    pos_mean_error = mean(abs(position_errors_m), 1);
    pos_max_error = max(abs(position_errors_m), [], 1);
    pos_rms_error = sqrt(mean(position_errors_m.^2, 1));
    
    fprintf('X axis - Mean Error: %.6f, Max Error: %.6f, RMS Error: %.6f\n', ...
            pos_mean_error(1), pos_max_error(1), pos_rms_error(1));
    fprintf('Y axis - Mean Error: %.6f, Max Error: %.6f, RMS Error: %.6f\n', ...
            pos_mean_error(2), pos_max_error(2), pos_rms_error(2));
    fprintf('Z axis - Mean Error: %.6f, Max Error: %.6f, RMS Error: %.6f\n', ...
            pos_mean_error(3), pos_max_error(3), pos_rms_error(3));
    
    fprintf('\n=== %s Position Error Statistics (mm) ===\n', arm_name);
    fprintf('X axis - Mean Error: %.3f, Max Error: %.3f, RMS Error: %.3f\n', ...
            pos_mean_error(1)*1000, pos_max_error(1)*1000, pos_rms_error(1)*1000);
    fprintf('Y axis - Mean Error: %.3f, Max Error: %.3f, RMS Error: %.3f\n', ...
            pos_mean_error(2)*1000, pos_max_error(2)*1000, pos_rms_error(2)*1000);
    fprintf('Z axis - Mean Error: %.3f, Max Error: %.3f, RMS Error: %.3f\n', ...
            pos_mean_error(3)*1000, pos_max_error(3)*1000, pos_rms_error(3)*1000);
    
    % Overall position error
    fprintf('\n=== %s Overall Position Error Statistics ===\n', arm_name);
    fprintf('Mean Total Error: %.6f m (%.3f mm)\n', mean(total_pos_error), mean(total_pos_error)*1000);
    fprintf('Max Total Error: %.6f m (%.3f mm)\n', max(total_pos_error), max(total_pos_error)*1000);
    fprintf('RMS Total Error: %.6f m (%.3f mm)\n', sqrt(mean(total_pos_error.^2)), sqrt(mean(total_pos_error.^2))*1000);
end


%% Save Results to CSV File
function save_results_to_csv(arm1_actual_joint_angles_rad, arm1_solved_joint_angles_rad, ...
                            arm1_joint_angle_errors_rad, arm1_joint_angle_errors_deg, ...
                            arm1_target_positions_m, arm1_calculated_positions_m, ...
                            arm1_position_errors_m, arm1_total_pos_error, arm1_convergence_flags, ...
                            arm2_actual_joint_angles_rad, arm2_solved_joint_angles_rad, ...
                            arm2_joint_angle_errors_rad, arm2_joint_angle_errors_deg, ...
                            arm2_target_positions_m, arm2_calculated_positions_m, ...
                            arm2_position_errors_m, arm2_total_pos_error, arm2_convergence_flags)

    result_filename = 'dual_arm_inverse_kinematics_result.csv';

    % Create variable names
    variable_names = {
        'Arm1_Actual_J1_rad', 'Arm1_Actual_J2_rad', 'Arm1_Actual_J3_rad', 'Arm1_Actual_J4_rad', ...
        'Arm1_Solved_J1_rad', 'Arm1_Solved_J2_rad', 'Arm1_Solved_J3_rad', 'Arm1_Solved_J4_rad', ...
        'Arm1_JointErr_J1_rad', 'Arm1_JointErr_J2_rad', 'Arm1_JointErr_J3_rad', 'Arm1_JointErr_J4_rad', ...
        'Arm1_JointErr_J1_deg', 'Arm1_JointErr_J2_deg', 'Arm1_JointErr_J3_deg', 'Arm1_JointErr_J4_deg', ...
        'Arm1_Target_X_m', 'Arm1_Target_Y_m', 'Arm1_Target_Z_m', ...
        'Arm1_Calc_X_m', 'Arm1_Calc_Y_m', 'Arm1_Calc_Z_m', ...
        'Arm1_PosErr_X_m', 'Arm1_PosErr_Y_m', 'Arm1_PosErr_Z_m', ...
        'Arm1_Total_Pos_Error_m', 'Arm1_Converged', ...
        'Arm2_Actual_J1_rad', 'Arm2_Actual_J2_rad', 'Arm2_Actual_J3_rad', 'Arm2_Actual_J4_rad', ...
        'Arm2_Solved_J1_rad', 'Arm2_Solved_J2_rad', 'Arm2_Solved_J3_rad', 'Arm2_Solved_J4_rad', ...
        'Arm2_JointErr_J1_rad', 'Arm2_JointErr_J2_rad', 'Arm2_JointErr_J3_rad', 'Arm2_JointErr_J4_rad', ...
        'Arm2_JointErr_J1_deg', 'Arm2_JointErr_J2_deg', 'Arm2_JointErr_J3_deg', 'Arm2_JointErr_J4_deg', ...
        'Arm2_Target_X_m', 'Arm2_Target_Y_m', 'Arm2_Target_Z_m', ...
        'Arm2_Calc_X_m', 'Arm2_Calc_Y_m', 'Arm2_Calc_Z_m', ...
        'Arm2_PosErr_X_m', 'Arm2_PosErr_Y_m', 'Arm2_PosErr_Z_m', ...
        'Arm2_Total_Pos_Error_m', 'Arm2_Converged'
    };

    % Create data table
    result_table = table( ...
    arm1_actual_joint_angles_rad(:,1), arm1_actual_joint_angles_rad(:,2), arm1_actual_joint_angles_rad(:,3), arm1_actual_joint_angles_rad(:,4), ...
    arm1_solved_joint_angles_rad(:,1), arm1_solved_joint_angles_rad(:,2), arm1_solved_joint_angles_rad(:,3), arm1_solved_joint_angles_rad(:,4), ...
    arm1_joint_angle_errors_rad(:,1), arm1_joint_angle_errors_rad(:,2), arm1_joint_angle_errors_rad(:,3), arm1_joint_angle_errors_rad(:,4), ...
    arm1_joint_angle_errors_deg(:,1), arm1_joint_angle_errors_deg(:,2), arm1_joint_angle_errors_deg(:,3), arm1_joint_angle_errors_deg(:,4), ...
    arm1_target_positions_m(:,1), arm1_target_positions_m(:,2), arm1_target_positions_m(:,3), ...
    arm1_calculated_positions_m(:,1), arm1_calculated_positions_m(:,2), arm1_calculated_positions_m(:,3), ...
    arm1_position_errors_m(:,1), arm1_position_errors_m(:,2), arm1_position_errors_m(:,3), ...
    arm1_total_pos_error, arm1_convergence_flags, ...
    arm2_actual_joint_angles_rad(:,1), arm2_actual_joint_angles_rad(:,2), arm2_actual_joint_angles_rad(:,3), arm2_actual_joint_angles_rad(:,4), ...
    arm2_solved_joint_angles_rad(:,1), arm2_solved_joint_angles_rad(:,2), arm2_solved_joint_angles_rad(:,3), arm2_solved_joint_angles_rad(:,4), ...
    arm2_joint_angle_errors_rad(:,1), arm2_joint_angle_errors_rad(:,2), arm2_joint_angle_errors_rad(:,3), arm2_joint_angle_errors_rad(:,4), ...
    arm2_joint_angle_errors_deg(:,1), arm2_joint_angle_errors_deg(:,2), arm2_joint_angle_errors_deg(:,3), arm2_joint_angle_errors_deg(:,4), ...
    arm2_target_positions_m(:,1), arm2_target_positions_m(:,2), arm2_target_positions_m(:,3), ...
    arm2_calculated_positions_m(:,1), arm2_calculated_positions_m(:,2), arm2_calculated_positions_m(:,3), ...
    arm2_position_errors_m(:,1), arm2_position_errors_m(:,2), arm2_position_errors_m(:,3), ...
    arm2_total_pos_error, arm2_convergence_flags, ...
    'VariableNames', variable_names);

    writetable(result_table, result_filename);
    fprintf('\nDual-arm analysis results saved to file: %s\n', result_filename);
end

%% Plot Dual Arm Error Charts
function create_dual_arm_plots(arm1_joint_errors_deg, arm1_position_errors_m, arm1_total_pos_error, arm1_convergence_flags, ...
                              arm2_joint_errors_deg, arm2_position_errors_m, arm2_total_pos_error, arm2_convergence_flags)

    % Create first figure: Arm 1 analysis
    figure('Name', 'Arm 1 Error Analysis', 'Position', [50, 50, 1400, 800]);
    create_single_arm_plots(arm1_joint_errors_deg, arm1_position_errors_m, arm1_total_pos_error, arm1_convergence_flags, 'Arm 1');
    saveas(gcf, 'arm1_inverse_kinematics_analysis.png');

    % Create second figure: Arm 2 analysis
    figure('Name', 'Arm 2 Error Analysis', 'Position', [100, 100, 1400, 800]);
    create_single_arm_plots(arm2_joint_errors_deg, arm2_position_errors_m, arm2_total_pos_error, arm2_convergence_flags, 'Arm 2');
    saveas(gcf, 'arm2_inverse_kinematics_analysis.png');

    % Create third figure: Comparison between both arms
    figure('Name', 'Dual Arm Comparison Analysis', 'Position', [150, 150, 1400, 800]);
    create_dual_arm_comparison_plots(arm1_joint_errors_deg, arm1_position_errors_m, arm1_total_pos_error, ...
                                    arm2_joint_errors_deg, arm2_position_errors_m, arm2_total_pos_error);
    saveas(gcf, 'dual_arm_comparison_analysis.png');

    fprintf('Dual-arm analysis plots saved\n');
end


%% Create Single Arm Plots
function create_single_arm_plots(joint_errors_deg, position_errors_m, total_pos_error, convergence_flags, arm_name)
    % Joint angle error plot
    subplot(2, 3, 1);
    plot(1:size(joint_errors_deg,1), joint_errors_deg(:,1), 'r-', 'LineWidth', 1);
    hold on;
    plot(1:size(joint_errors_deg,1), joint_errors_deg(:,2), 'g-', 'LineWidth', 1);
    plot(1:size(joint_errors_deg,1), joint_errors_deg(:,3), 'b-', 'LineWidth', 1);
    plot(1:size(joint_errors_deg,1), joint_errors_deg(:,4), 'm-', 'LineWidth', 1);
    xlabel('Data Points');
    ylabel('Joint Angle Error (deg)');
    title(sprintf('%s Joint Angle Error', arm_name));
    legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4');
    grid on;

    % Position error plot
    subplot(2, 3, 2);
    plot(1:size(position_errors_m,1), position_errors_m(:,1)*1000, 'r-', 'LineWidth', 1);
    hold on;
    plot(1:size(position_errors_m,1), position_errors_m(:,2)*1000, 'g-', 'LineWidth', 1);
    plot(1:size(position_errors_m,1), position_errors_m(:,3)*1000, 'b-', 'LineWidth', 1);
    xlabel('Data Points');
    ylabel('Position Error (mm)');
    title(sprintf('%s Position Error', arm_name));
    legend('X Error', 'Y Error', 'Z Error');
    grid on;

    % Total position error plot
    subplot(2, 3, 3);
    plot(1:length(total_pos_error), total_pos_error*1000, 'k-', 'LineWidth', 1.5);
    xlabel('Data Points');
    ylabel('Total Position Error (mm)');
    title(sprintf('%s Total Position Error', arm_name));
    grid on;

    % Joint angle error distribution
    subplot(2, 3, 4);
    joint_total_error = sqrt(sum(joint_errors_deg.^2, 2));
    histogram(joint_total_error, 50);
    xlabel('Total Joint Error (deg)');
    ylabel('Frequency');
    title(sprintf('%s Joint Angle Error Distribution', arm_name));
    grid on;

    % Position error distribution
    subplot(2, 3, 5);
    histogram(total_pos_error*1000, 50);
    xlabel('Total Position Error (mm)');
    ylabel('Frequency');
    title(sprintf('%s Position Error Distribution', arm_name));
    grid on;

    % Convergence flags
    subplot(2, 3, 6);
    plot(1:length(convergence_flags), convergence_flags, 'bo-', 'MarkerSize', 3);
    xlabel('Data Points');
    ylabel('Convergence Flag');
    title(sprintf('%s Convergence Status', arm_name));
    ylim([-0.1, 1.1]);
    grid on;
end

%% Create Dual Arm Comparison Plots
function create_dual_arm_comparison_plots(arm1_joint_errors_deg, arm1_position_errors_m, arm1_total_pos_error, ...
                                         arm2_joint_errors_deg, arm2_position_errors_m, arm2_total_pos_error)

    % Joint angle error comparison - Joints 1 and 2
    subplot(2, 3, 1);
    plot(1:size(arm1_joint_errors_deg,1), arm1_joint_errors_deg(:,1), 'r-', 'LineWidth', 1);
    hold on;
    plot(1:size(arm1_joint_errors_deg,1), arm1_joint_errors_deg(:,2), 'g-', 'LineWidth', 1);
    plot(1:size(arm2_joint_errors_deg,1), arm2_joint_errors_deg(:,1), 'r--', 'LineWidth', 1);
    plot(1:size(arm2_joint_errors_deg,1), arm2_joint_errors_deg(:,2), 'g--', 'LineWidth', 1);
    xlabel('Data Points');
    ylabel('Joint Angle Error (deg)');
    title('Joint 1 and 2 Error Comparison');
    legend('Arm1-Joint1', 'Arm1-Joint2', 'Arm2-Joint1', 'Arm2-Joint2');
    grid on;

    % Joint angle error comparison - Joints 3 and 4
    subplot(2, 3, 2);
    plot(1:size(arm1_joint_errors_deg,1), arm1_joint_errors_deg(:,3), 'b-', 'LineWidth', 1);
    hold on;
    plot(1:size(arm1_joint_errors_deg,1), arm1_joint_errors_deg(:,4), 'm-', 'LineWidth', 1);
    plot(1:size(arm2_joint_errors_deg,1), arm2_joint_errors_deg(:,3), 'b--', 'LineWidth', 1);
    plot(1:size(arm2_joint_errors_deg,1), arm2_joint_errors_deg(:,4), 'm--', 'LineWidth', 1);
    xlabel('Data Points');
    ylabel('Joint Angle Error (deg)');
    title('Joint 3 and 4 Error Comparison');
    legend('Arm1-Joint3', 'Arm1-Joint4', 'Arm2-Joint3', 'Arm2-Joint4');
    grid on;

    % Position error comparison - X and Y axes
    subplot(2, 3, 3);
    plot(1:size(arm1_position_errors_m,1), arm1_position_errors_m(:,1)*1000, 'r-', 'LineWidth', 1);
    hold on;
    plot(1:size(arm1_position_errors_m,1), arm1_position_errors_m(:,2)*1000, 'g-', 'LineWidth', 1);
    plot(1:size(arm2_position_errors_m,1), arm2_position_errors_m(:,1)*1000, 'r--', 'LineWidth', 1);
    plot(1:size(arm2_position_errors_m,1), arm2_position_errors_m(:,2)*1000, 'g--', 'LineWidth', 1);
    xlabel('Data Points');
    ylabel('Position Error (mm)');
    title('X and Y Axis Position Error Comparison');
    legend('Arm1-X', 'Arm1-Y', 'Arm2-X', 'Arm2-Y');
    grid on;

    % Position error comparison - Z axis
    subplot(2, 3, 4);
    plot(1:size(arm1_position_errors_m,1), arm1_position_errors_m(:,3)*1000, 'b-', 'LineWidth', 1);
    hold on;
    plot(1:size(arm2_position_errors_m,1), arm2_position_errors_m(:,3)*1000, 'b--', 'LineWidth', 1);
    xlabel('Data Points');
    ylabel('Position Error (mm)');
    title('Z Axis Position Error Comparison');
    legend('Arm1-Z', 'Arm2-Z');
    grid on;

    % Total position error comparison
    subplot(2, 3, 5);
    plot(1:length(arm1_total_pos_error), arm1_total_pos_error*1000, 'k-', 'LineWidth', 1.5);
    hold on;
    plot(1:length(arm2_total_pos_error), arm2_total_pos_error*1000, 'k--', 'LineWidth', 1.5);
    xlabel('Data Points');
    ylabel('Total Position Error (mm)');
    title('Total Position Error Comparison');
    legend('Arm1', 'Arm2');
    grid on;

    % Position error distribution comparison
    subplot(2, 3, 6);
    histogram(arm1_total_pos_error*1000, 30, 'FaceAlpha', 0.7, 'FaceColor', 'blue');
    hold on;
    histogram(arm2_total_pos_error*1000, 30, 'FaceAlpha', 0.7, 'FaceColor', 'red');
    xlabel('Total Position Error (mm)');
    ylabel('Frequency');
    title('Total Position Error Distribution Comparison');
    legend('Arm1', 'Arm2');
    grid on;
end
