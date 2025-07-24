function ik_iterative(target_positions)
% Dual robotic arm inverse kinematics analysis function
% Input: target_positions - 1×6 array [x1,y1,z1,x2,y2,z2]
%        First three are end-effector coordinates xyz for robotic arm 1 (unit: m)
%        Last three are end-effector coordinates xyz for robotic arm 2 (unit: m)
% Output: arm1_data, arm2_data - stored in workspace for Simulink calls

% Check input parameters
if nargin < 1
    error('Please provide target position parameters! Format: [x1,y1,z1,x2,y2,z2]');
end

if length(target_positions) ~= 6
    error('Input parameter must be a 1×6 array! Format: [x1,y1,z1,x2,y2,z2]');
end

% Separate target positions for both robotic arms
arm1_target = target_positions(1:3);  % Target position for robotic arm 1 (m)
arm2_target = target_positions(4:6);  % Target position for robotic arm 2 (m)

fprintf('=== Dual Robotic Arm Inverse Kinematics Analysis ===\n');
fprintf('Robotic Arm 1 target position: [%.3f, %.3f, %.3f] m\n', arm1_target);
fprintf('Robotic Arm 2 target position: [%.3f, %.3f, %.3f] m\n', arm2_target);

% DH parameter table for robotic arm 1
% DH parameters: [theta_initial, d, a, alpha] (angles in degrees, lengths in mm)
DH_params_arm1 = [
    0,    109,   5.1,  -90;  % Transform from world frame to joint 1 (fixed)
    0,    5.3,   30.9,  90;  % Transform from joint 1 to joint 2
    0,    0,     270,  -90;  % Transform from joint 2 to joint 3
    90,   22.8,  0,     90;  % Transform from joint 3 to joint 4
    0,    265,   0,     0    % Transform from joint 4 to joint 5
];

% DH parameter table for Arm 2
DH_params_arm2 = [
    0,    109,   354.9,  90;  % Transform from world frame to joint 1 (fixed)
    0,    -128.8, 30.9, -90;  % Transform from joint 1 to joint 2
    0,    0,      270,   90;  % Transform from joint 2 to joint 3
    90,   22.8,      0,     90;  % Transform from joint 3 to joint 4
    0,    265,    0,     0    % Transform from joint 4 to joint 5
];

% Inverse kinematics solver parameters
options = optimset('Display', 'off', 'TolFun', 1e-6, 'TolX', 1e-6, 'MaxIter', 1000);

% Initial guess values (can be adjusted based on actual conditions)
initial_guess_arm1 = [0, 0, 0, 0];  % Initial joint angles for robotic arm 1
initial_guess_arm2 = [0, 0, 0, 0];  % Initial joint angles for robotic arm 2

%% Solve inverse kinematics for robotic arm 1
fprintf('\nSolving inverse kinematics for robotic arm 1...\n');
arm1_target_mm = arm1_target * 1000;  % Convert to mm

% Define objective function for robotic arm 1
objective_function_arm1 = @(joint_angles) inverse_kinematics_objective(joint_angles, arm1_target_mm, DH_params_arm1);

% Solve inverse kinematics for robotic arm 1
try
    [arm1_joint_angles, fval1, exitflag1] = fminsearch(objective_function_arm1, initial_guess_arm1, options);
    arm1_converged = (exitflag1 == 1 && fval1 < 1e-3);
    
    if arm1_converged
        fprintf('Robotic arm 1 inverse kinematics solved successfully!\n');
        fprintf('Joint angles (radians): [%.6f, %.6f, %.6f, %.6f]\n', arm1_joint_angles);
        fprintf('Joint angles (degrees): [%.2f, %.2f, %.2f, %.2f]\n', arm1_joint_angles * 180 / pi);
    else
        fprintf('Robotic arm 1 inverse kinematics did not converge, error: %.6f\n', fval1);
    end
catch ME
    fprintf('Robotic arm 1 inverse kinematics solution failed: %s\n', ME.message);
    arm1_joint_angles = initial_guess_arm1;
    arm1_converged = false;
end

%% Solve inverse kinematics for robotic arm 2
fprintf('\nSolving inverse kinematics for robotic arm 2...\n');
arm2_target_mm = arm2_target * 1000;  % Convert to mm

% Define objective function for robotic arm 2
objective_function_arm2 = @(joint_angles) inverse_kinematics_objective(joint_angles, arm2_target_mm, DH_params_arm2);

% Solve inverse kinematics for robotic arm 2
try
    [arm2_joint_angles, fval2, exitflag2] = fminsearch(objective_function_arm2, initial_guess_arm2, options);
    arm2_converged = (exitflag2 == 1 && fval2 < 1e-3);
    
    if arm2_converged
        fprintf('Robotic arm 2 inverse kinematics solved successfully!\n');
        fprintf('Joint angles (radians): [%.6f, %.6f, %.6f, %.6f]\n', arm2_joint_angles);
        fprintf('Joint angles (degrees): [%.2f, %.2f, %.2f, %.2f]\n', arm2_joint_angles * 180 / pi);
    else
        fprintf('Robotic arm 2 inverse kinematics did not converge, error: %.6f\n', fval2);
    end
catch ME
    fprintf('Robotic arm 2 inverse kinematics solution failed: %s\n', ME.message);
    arm2_joint_angles = initial_guess_arm2;
    arm2_converged = false;
end

%% Verify solution results
fprintf('\n=== Forward Kinematics Verification ===\n');

% Verify robotic arm 1
T1_final = forward_kinematics(arm1_joint_angles, DH_params_arm1);
arm1_calculated_pos = T1_final(1:3, 4)' / 1000;  % Convert to m
arm1_pos_error = arm1_calculated_pos - arm1_target;
arm1_total_error = norm(arm1_pos_error);

fprintf('Robotic Arm 1:\n');
fprintf('  Target position: [%.6f, %.6f, %.6f] m\n', arm1_target);
fprintf('  Calculated position: [%.6f, %.6f, %.6f] m\n', arm1_calculated_pos);
fprintf('  Position error: [%.6f, %.6f, %.6f] m\n', arm1_pos_error);
fprintf('  Total error: %.6f m (%.3f mm)\n', arm1_total_error, arm1_total_error*1000);

% Verify robotic arm 2
T2_final = forward_kinematics(arm2_joint_angles, DH_params_arm2);
arm2_calculated_pos = T2_final(1:3, 4)' / 1000;  % Convert to m
arm2_pos_error = arm2_calculated_pos - arm2_target;
arm2_total_error = norm(arm2_pos_error);

fprintf('Robotic Arm 2:\n');
fprintf('  Target position: [%.6f, %.6f, %.6f] m\n', arm2_target);
fprintf('  Calculated position: [%.6f, %.6f, %.6f] m\n', arm2_calculated_pos);
fprintf('  Position error: [%.6f, %.6f, %.6f] m\n', arm2_pos_error);
fprintf('  Total error: %.6f m (%.3f mm)\n', arm2_total_error, arm2_total_error*1000);

%% Create Simulink data structure
% Create time vector (assumes single point data here, can be extended to time series for actual use)
time_vector = [0, 1];  % Simple time vector

% Create arm1_data structure (90x5 double format, adapted for Simulink's From Workspace block)
arm1_data = struct();
arm1_data.time = time_vector';
arm1_data.signals = struct();
arm1_data.signals.values = [0,0,0,0,0; arm1_joint_angles,0];  % 4 joint angles + 1 additional column
arm1_data.signals.dimensions = 5;

% Create arm2_data structure
arm2_data = struct();
arm2_data.time = time_vector';
arm2_data.signals = struct();
arm2_data.signals.values = [0,0,0,0,0; arm2_joint_angles,0];  % 4 joint angles + 1 additional column
arm2_data.signals.dimensions = 5;

%% Store data to workspace
assignin('base', 'arm1_data', arm1_data);
assignin('base', 'arm2_data', arm2_data);

fprintf('\n=== Data stored to workspace ===\n');
fprintf('Variable names: arm1_data, arm2_data\n');
fprintf('Data format: Time series structure, suitable for Simulink From Workspace block\n');
fprintf('arm1_data joint angles (radians): [%.6f, %.6f, %.6f, %.6f]\n', arm1_joint_angles);
fprintf('arm2_data joint angles (radians): [%.6f, %.6f, %.6f, %.6f]\n', arm2_joint_angles);

% Display usage instructions
fprintf('\n=== Simulink Usage Instructions ===\n');
fprintf('1. Add From Workspace block to Simulink model\n');
fprintf('2. Enter variable name in block parameters: arm1_data or arm2_data\n');
fprintf('3. Set output data type to: Inherit: auto\n');
fprintf('4. Set sample time to: -1 (inherit)\n');
fprintf('5. Interpolate data and enable zero-crossing detection can be set as needed\n');

end

%% Inverse kinematics objective function
function error = inverse_kinematics_objective(joint_angles, target_position_mm, DH_params)
    % Objective function: minimize end-effector position error
    % Input: joint_angles - joint angles (radians)
    %        target_position_mm - target position (mm)
    %        DH_params - DH parameter table
    % Output: error - sum of squared position errors
    
    % Calculate forward kinematics
    T_final = forward_kinematics(joint_angles, DH_params);
    
    % Extract calculated position
    calculated_position_mm = T_final(1:3, 4);
    
    % Calculate position error
    position_error = calculated_position_mm' - target_position_mm;
    
    % Return sum of squared errors
    error = sum(position_error.^2);
end

%% Forward kinematics calculation function
function T_final = forward_kinematics(joint_angles_rad, DH_params_initial)
    % Input: joint_angles_rad - 4 joint rotation angles (radians)
    %        DH_params_initial - DH parameter matrix (angles in degrees, lengths in mm)
    % Output: T_final - final transformation matrix (position units in mm)
    
    % Initialize as identity matrix
    T_final = eye(4);
    
    % Calculate transformation matrix for each joint
    for i = 1:size(DH_params_initial, 1)
        if i == 1
            % First row: transformation from world frame to joint 1 (fixed transformation)
            theta_rad = DH_params_initial(i, 1) * pi / 180;
        else
            % Second row and beyond: transformation from joint i-1 to joint i
            % Use initial angle from DH table + joint rotation angle
            theta_initial_rad = DH_params_initial(i, 1) * pi / 180;
            joint_index = i - 1;  % Joint index (second row corresponds to joint 1)
            
            if joint_index <= length(joint_angles_rad)
                theta_rad = theta_initial_rad + joint_angles_rad(joint_index);
            else
                % If no corresponding joint angle, use initial angle
                theta_rad = theta_initial_rad;
            end
        end
        
        d_mm = DH_params_initial(i, 2);
        a_mm = DH_params_initial(i, 3);
        alpha_rad = DH_params_initial(i, 4) * pi / 180;
        
        % Calculate transformation matrix for single joint
        T_i = dh_transform(theta_rad, d_mm, a_mm, alpha_rad);
        
        % Accumulate transformation
        T_final = T_final * T_i;
    end
end

%% DH transformation matrix calculation function
function T = dh_transform(theta, d, a, alpha)
    % Calculate transformation matrix based on DH parameters
    % Input: theta, d, a, alpha - DH parameters
    % Output: T - 4x4 transformation matrix
    
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end