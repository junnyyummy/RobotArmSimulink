% 双机械臂逆运动学数值解法与误差分析程序
% 从CSV文件读取末端位置，求解逆运动学，分析关节角度误差和位置误差

clc;
clear;
close all;

% 设置CSV文件路径
logFilePath = 'joint_log.csv';

% 检查CSV文件是否存在
if ~isfile(logFilePath)
    error('CSV文件不存在，请确保joint_log.csv在工作目录下');
end

% 读取CSV文件
try
    data = readtable(logFilePath);
    fprintf('成功读取CSV文件，共有%d行数据\n', height(data));
catch ME
    error('读取CSV文件失败: %s', ME.message);
end

% 获取数据列数，确保有足够的列
if width(data) < 20
    error('CSV文件列数不足，需要至少20列数据');
end

% 提取数据
% 机械臂1数据
arm1_actual_joint_angles_rad = table2array(data(:, 1:4));    % 实际关节角度（弧度）
arm1_target_positions_m = table2array(data(:, 9:11));        % 目标末端位置（x,y,z，单位：m）

% 机械臂2数据
arm2_actual_joint_angles_rad = table2array(data(:, 5:8));    % 实际关节角度（弧度）
arm2_target_positions_m = table2array(data(:, 15:17));       % 目标末端位置（x,y,z，单位：m）

% 机械臂1的DH参数表
DH_params_arm1 = [
    0,    109,   5.1,  -90;  % 世界坐标系到关节1的转换（固定）
    0,    5.3,   30.9,  90;  % 关节1到关节2的变换
    0,    0,     270,  -90;  % 关节2到关节3的变换
    90,   22.8,  0,     90;  % 关节3到关节4的变换
    0,    250,   0,     0    % 关节4到关节5的变换
];

% 机械臂2的DH参数表
DH_params_arm2 = [
    0,    109,   354.9,  90;  % 世界坐标系到关节1的转换（固定）
    0,    -128.8, 30.9, -90;  % 关节1到关节2的变换
    0,    0,      270,   90;  % 关节2到关节3的变换
    90,   22.8,      0,     90;  % 关节3到关节4的变换
    0,    265,    0,     0    % 关节4到关节5的变换
];

% 初始化结果存储
num_points = size(arm1_target_positions_m, 1);

% 机械臂1结果存储
arm1_solved_joint_angles_rad = zeros(num_points, 4);
arm1_joint_angle_errors_rad = zeros(num_points, 4);
arm1_joint_angle_errors_deg = zeros(num_points, 4);
arm1_calculated_positions_m = zeros(num_points, 3);
arm1_position_errors_m = zeros(num_points, 3);
arm1_convergence_flags = zeros(num_points, 1);

% 机械臂2结果存储
arm2_solved_joint_angles_rad = zeros(num_points, 4);
arm2_joint_angle_errors_rad = zeros(num_points, 4);
arm2_joint_angle_errors_deg = zeros(num_points, 4);
arm2_calculated_positions_m = zeros(num_points, 3);
arm2_position_errors_m = zeros(num_points, 3);
arm2_convergence_flags = zeros(num_points, 1);

% 逆运动学求解参数
options = optimset('Display', 'off', 'TolFun', 1e-6, 'TolX', 1e-6, 'MaxIter', 500);

fprintf('开始双机械臂逆运动学求解...\n');

% 对每一个目标位置进行逆运动学求解
for i = 1:num_points
    % === 机械臂1求解 ===
    target_pos_mm_arm1 = arm1_target_positions_m(i, :) * 1000;
    initial_guess_arm1 = arm1_actual_joint_angles_rad(i, :);
    
    objective_function_arm1 = @(joint_angles) inverse_kinematics_objective(joint_angles, target_pos_mm_arm1, DH_params_arm1);
    
    try
        [solved_angles_arm1, fval_arm1, exitflag_arm1] = fminsearch(objective_function_arm1, initial_guess_arm1, options);
        arm1_solved_joint_angles_rad(i, :) = solved_angles_arm1;
        arm1_convergence_flags(i) = (exitflag_arm1 == 1 && fval_arm1 < 1e-3);
    catch ME
        fprintf('机械臂1数据点 %d 求解失败: %s\n', i, ME.message);
        arm1_solved_joint_angles_rad(i, :) = initial_guess_arm1;
        arm1_convergence_flags(i) = 0;
    end
    
    % 计算机械臂1关节角度误差
    arm1_joint_angle_errors_rad(i, :) = arm1_solved_joint_angles_rad(i, :) - arm1_actual_joint_angles_rad(i, :);
    arm1_joint_angle_errors_deg(i, :) = arm1_joint_angle_errors_rad(i, :) * 180 / pi;
    
    % 机械臂1正运动学验证
    T_final_arm1 = forward_kinematics(arm1_solved_joint_angles_rad(i, :), DH_params_arm1);
    calculated_pos_mm_arm1 = T_final_arm1(1:3, 4);
    arm1_calculated_positions_m(i, :) = calculated_pos_mm_arm1' / 1000;
    arm1_position_errors_m(i, :) = arm1_calculated_positions_m(i, :) - arm1_target_positions_m(i, :);
    
    % === 机械臂2求解 ===
    target_pos_mm_arm2 = arm2_target_positions_m(i, :) * 1000;
    initial_guess_arm2 = arm2_actual_joint_angles_rad(i, :);
    
    objective_function_arm2 = @(joint_angles) inverse_kinematics_objective(joint_angles, target_pos_mm_arm2, DH_params_arm2);
    
    try
        [solved_angles_arm2, fval_arm2, exitflag_arm2] = fminsearch(objective_function_arm2, initial_guess_arm2, options);
        arm2_solved_joint_angles_rad(i, :) = solved_angles_arm2;
        arm2_convergence_flags(i) = (exitflag_arm2 == 1 && fval_arm2 < 1e-3);
    catch ME
        fprintf('机械臂2数据点 %d 求解失败: %s\n', i, ME.message);
        arm2_solved_joint_angles_rad(i, :) = initial_guess_arm2;
        arm2_convergence_flags(i) = 0;
    end
    
    % 计算机械臂2关节角度误差
    arm2_joint_angle_errors_rad(i, :) = arm2_solved_joint_angles_rad(i, :) - arm2_actual_joint_angles_rad(i, :);
    arm2_joint_angle_errors_deg(i, :) = arm2_joint_angle_errors_rad(i, :) * 180 / pi;
    
    % 机械臂2正运动学验证
    T_final_arm2 = forward_kinematics(arm2_solved_joint_angles_rad(i, :), DH_params_arm2);
    calculated_pos_mm_arm2 = T_final_arm2(1:3, 4);
    arm2_calculated_positions_m(i, :) = calculated_pos_mm_arm2' / 1000;
    arm2_position_errors_m(i, :) = arm2_calculated_positions_m(i, :) - arm2_target_positions_m(i, :);
    
    % 显示进度
    if mod(i, 100) == 0 || i == num_points
        fprintf('已处理 %d/%d 个数据点\n', i, num_points);
    end
end

% === 统计结果 ===
% 机械臂1统计
arm1_converged_count = sum(arm1_convergence_flags);
arm1_total_pos_error = sqrt(sum(arm1_position_errors_m.^2, 2));

% 机械臂2统计
arm2_converged_count = sum(arm2_convergence_flags);
arm2_total_pos_error = sqrt(sum(arm2_position_errors_m.^2, 2));

% 显示统计结果
print_statistics('机械臂1', arm1_converged_count, num_points, arm1_joint_angle_errors_rad, ...
                 arm1_joint_angle_errors_deg, arm1_position_errors_m, arm1_total_pos_error);

print_statistics('机械臂2', arm2_converged_count, num_points, arm2_joint_angle_errors_rad, ...
                 arm2_joint_angle_errors_deg, arm2_position_errors_m, arm2_total_pos_error);

% 保存结果到CSV文件
save_results_to_csv(arm1_actual_joint_angles_rad, arm1_solved_joint_angles_rad, ...
                   arm1_joint_angle_errors_rad, arm1_joint_angle_errors_deg, ...
                   arm1_target_positions_m, arm1_calculated_positions_m, ...
                   arm1_position_errors_m, arm1_total_pos_error, arm1_convergence_flags, ...
                   arm2_actual_joint_angles_rad, arm2_solved_joint_angles_rad, ...
                   arm2_joint_angle_errors_rad, arm2_joint_angle_errors_deg, ...
                   arm2_target_positions_m, arm2_calculated_positions_m, ...
                   arm2_position_errors_m, arm2_total_pos_error, arm2_convergence_flags);

% 绘制误差图表
create_dual_arm_plots(arm1_joint_angle_errors_deg, arm1_position_errors_m, arm1_total_pos_error, arm1_convergence_flags, ...
                     arm2_joint_angle_errors_deg, arm2_position_errors_m, arm2_total_pos_error, arm2_convergence_flags);

fprintf('\n双机械臂逆运动学分析程序执行完成！\n');

Ts = 1; % 示例采样时间
time_vector = (0:num_points-1)' * Ts;

% 组合成带时间戳的数据矩阵（时间列 + 4个关节列）
arm1_data = [time_vector, arm1_solved_joint_angles_rad];
arm2_data = [time_vector, arm2_solved_joint_angles_rad];

%% 逆运动学目标函数
function error = inverse_kinematics_objective(joint_angles, target_position_mm, DH_params)
    % 目标函数：最小化末端位置误差
    T_final = forward_kinematics(joint_angles, DH_params);
    calculated_position_mm = T_final(1:3, 4);
    position_error = calculated_position_mm' - target_position_mm;
    error = sum(position_error.^2);
end

%% 正运动学计算函数
function T_final = forward_kinematics(joint_angles_rad, DH_params_initial)
    % 正运动学计算
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

%% DH变换矩阵计算函数
function T = dh_transform(theta, d, a, alpha)
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end

%% 打印统计结果函数
function print_statistics(arm_name, converged_count, num_points, joint_angle_errors_rad, ...
                         joint_angle_errors_deg, position_errors_m, total_pos_error)
    fprintf('\n=== %s 逆运动学求解完成 ===\n', arm_name);
    fprintf('处理数据点数量: %d\n', num_points);
    fprintf('收敛数据点数量: %d (%.1f%%)\n', converged_count, 100*converged_count/num_points);
    
    % 关节角度误差统计（弧度）
    fprintf('\n=== %s 关节角度误差统计 (弧度) ===\n', arm_name);
    joint_mean_error_rad = mean(abs(joint_angle_errors_rad), 1);
    joint_max_error_rad = max(abs(joint_angle_errors_rad), [], 1);
    joint_rms_error_rad = sqrt(mean(joint_angle_errors_rad.^2, 1));
    
    for j = 1:4
        fprintf('关节%d - 平均误差: %.6f, 最大误差: %.6f, RMS误差: %.6f\n', ...
                j, joint_mean_error_rad(j), joint_max_error_rad(j), joint_rms_error_rad(j));
    end
    
    % 关节角度误差统计（度）
    fprintf('\n=== %s 关节角度误差统计 (度) ===\n', arm_name);
    joint_mean_error_deg = mean(abs(joint_angle_errors_deg), 1);
    joint_max_error_deg = max(abs(joint_angle_errors_deg), [], 1);
    joint_rms_error_deg = sqrt(mean(joint_angle_errors_deg.^2, 1));
    
    for j = 1:4
        fprintf('关节%d - 平均误差: %.3f, 最大误差: %.3f, RMS误差: %.3f\n', ...
                j, joint_mean_error_deg(j), joint_max_error_deg(j), joint_rms_error_deg(j));
    end
    
    % 位置误差统计
    fprintf('\n=== %s 位置误差统计 (m) ===\n', arm_name);
    pos_mean_error = mean(abs(position_errors_m), 1);
    pos_max_error = max(abs(position_errors_m), [], 1);
    pos_rms_error = sqrt(mean(position_errors_m.^2, 1));
    
    fprintf('X轴 - 平均误差: %.6f, 最大误差: %.6f, RMS误差: %.6f\n', ...
            pos_mean_error(1), pos_max_error(1), pos_rms_error(1));
    fprintf('Y轴 - 平均误差: %.6f, 最大误差: %.6f, RMS误差: %.6f\n', ...
            pos_mean_error(2), pos_max_error(2), pos_rms_error(2));
    fprintf('Z轴 - 平均误差: %.6f, 最大误差: %.6f, RMS误差: %.6f\n', ...
            pos_mean_error(3), pos_max_error(3), pos_rms_error(3));
    
    fprintf('\n=== %s 位置误差统计 (mm) ===\n', arm_name);
    fprintf('X轴 - 平均误差: %.3f, 最大误差: %.3f, RMS误差: %.3f\n', ...
            pos_mean_error(1)*1000, pos_max_error(1)*1000, pos_rms_error(1)*1000);
    fprintf('Y轴 - 平均误差: %.3f, 最大误差: %.3f, RMS误差: %.3f\n', ...
            pos_mean_error(2)*1000, pos_max_error(2)*1000, pos_rms_error(2)*1000);
    fprintf('Z轴 - 平均误差: %.3f, 最大误差: %.3f, RMS误差: %.3f\n', ...
            pos_mean_error(3)*1000, pos_max_error(3)*1000, pos_rms_error(3)*1000);
    
    % 总体位置误差
    fprintf('\n=== %s 总体位置误差统计 ===\n', arm_name);
    fprintf('平均总体误差: %.6f m (%.3f mm)\n', mean(total_pos_error), mean(total_pos_error)*1000);
    fprintf('最大总体误差: %.6f m (%.3f mm)\n', max(total_pos_error), max(total_pos_error)*1000);
    fprintf('RMS总体误差: %.6f m (%.3f mm)\n', sqrt(mean(total_pos_error.^2)), sqrt(mean(total_pos_error.^2))*1000);
end

%% 保存结果到CSV文件
function save_results_to_csv(arm1_actual_joint_angles_rad, arm1_solved_joint_angles_rad, ...
                            arm1_joint_angle_errors_rad, arm1_joint_angle_errors_deg, ...
                            arm1_target_positions_m, arm1_calculated_positions_m, ...
                            arm1_position_errors_m, arm1_total_pos_error, arm1_convergence_flags, ...
                            arm2_actual_joint_angles_rad, arm2_solved_joint_angles_rad, ...
                            arm2_joint_angle_errors_rad, arm2_joint_angle_errors_deg, ...
                            arm2_target_positions_m, arm2_calculated_positions_m, ...
                            arm2_position_errors_m, arm2_total_pos_error, arm2_convergence_flags)
    
    result_filename = 'dual_arm_inverse_kinematics_result.csv';
    
    % 创建变量名
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
    
    % 创建数据表
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
    fprintf('\n双机械臂分析结果已保存到文件: %s\n', result_filename);
end

%% 绘制双机械臂误差图表
function create_dual_arm_plots(arm1_joint_errors_deg, arm1_position_errors_m, arm1_total_pos_error, arm1_convergence_flags, ...
                              arm2_joint_errors_deg, arm2_position_errors_m, arm2_total_pos_error, arm2_convergence_flags)
    
    % 创建第一个图形：机械臂1分析
    figure('Name', '机械臂1误差分析', 'Position', [50, 50, 1400, 800]);
    create_single_arm_plots(arm1_joint_errors_deg, arm1_position_errors_m, arm1_total_pos_error, arm1_convergence_flags, '机械臂1');
    saveas(gcf, 'arm1_inverse_kinematics_analysis.png');
    
    % 创建第二个图形：机械臂2分析
    figure('Name', '机械臂2误差分析', 'Position', [100, 100, 1400, 800]);
    create_single_arm_plots(arm2_joint_errors_deg, arm2_position_errors_m, arm2_total_pos_error, arm2_convergence_flags, '机械臂2');
    saveas(gcf, 'arm2_inverse_kinematics_analysis.png');
    
    % 创建第三个图形：双机械臂对比
    figure('Name', '双机械臂对比分析', 'Position', [150, 150, 1400, 800]);
    create_dual_arm_comparison_plots(arm1_joint_errors_deg, arm1_position_errors_m, arm1_total_pos_error, ...
                                    arm2_joint_errors_deg, arm2_position_errors_m, arm2_total_pos_error);
    saveas(gcf, 'dual_arm_comparison_analysis.png');
    
    fprintf('双机械臂分析图表已保存\n');
end

%% 创建单个机械臂的图表
function create_single_arm_plots(joint_errors_deg, position_errors_m, total_pos_error, convergence_flags, arm_name)
    % 关节角度误差图
    subplot(2, 3, 1);
    plot(1:size(joint_errors_deg,1), joint_errors_deg(:,1), 'r-', 'LineWidth', 1);
    hold on;
    plot(1:size(joint_errors_deg,1), joint_errors_deg(:,2), 'g-', 'LineWidth', 1);
    plot(1:size(joint_errors_deg,1), joint_errors_deg(:,3), 'b-', 'LineWidth', 1);
    plot(1:size(joint_errors_deg,1), joint_errors_deg(:,4), 'm-', 'LineWidth', 1);
    xlabel('数据点');
    ylabel('关节角度误差 (度)');
    title(sprintf('%s 关节角度误差', arm_name));
    legend('关节1', '关节2', '关节3', '关节4');
    grid on;
    
    % 位置误差图
    subplot(2, 3, 2);
    plot(1:size(position_errors_m,1), position_errors_m(:,1)*1000, 'r-', 'LineWidth', 1);
    hold on;
    plot(1:size(position_errors_m,1), position_errors_m(:,2)*1000, 'g-', 'LineWidth', 1);
    plot(1:size(position_errors_m,1), position_errors_m(:,3)*1000, 'b-', 'LineWidth', 1);
    xlabel('数据点');
    ylabel('位置误差 (mm)');
    title(sprintf('%s 位置误差', arm_name));
    legend('X轴误差', 'Y轴误差', 'Z轴误差');
    grid on;
    
    % 总体位置误差图
    subplot(2, 3, 3);
    plot(1:length(total_pos_error), total_pos_error*1000, 'k-', 'LineWidth', 1.5);
    xlabel('数据点');
    ylabel('总体位置误差 (mm)');
    title(sprintf('%s 总体位置误差', arm_name));
    grid on;
    
    % 关节角度误差分布
    subplot(2, 3, 4);
    joint_total_error = sqrt(sum(joint_errors_deg.^2, 2));
    histogram(joint_total_error, 50);
    xlabel('关节角度总误差 (度)');
    ylabel('频次');
    title(sprintf('%s 关节角度误差分布', arm_name));
    grid on;
    
    % 位置误差分布
    subplot(2, 3, 5);
    histogram(total_pos_error*1000, 50);
    xlabel('总体位置误差 (mm)');
    ylabel('频次');
    title(sprintf('%s 位置误差分布', arm_name));
    grid on;
    
    % 收敛情况
    subplot(2, 3, 6);
    plot(1:length(convergence_flags), convergence_flags, 'bo-', 'MarkerSize', 3);
    xlabel('数据点');
    ylabel('收敛标志');
    title(sprintf('%s 收敛情况', arm_name));
    ylim([-0.1, 1.1]);
    grid on;
end

%% 创建双机械臂对比图表
function create_dual_arm_comparison_plots(arm1_joint_errors_deg, arm1_position_errors_m, arm1_total_pos_error, ...
                                         arm2_joint_errors_deg, arm2_position_errors_m, arm2_total_pos_error)
    
    % 关节角度误差对比 - 关节1和关节2
    subplot(2, 3, 1);
    plot(1:size(arm1_joint_errors_deg,1), arm1_joint_errors_deg(:,1), 'r-', 'LineWidth', 1);
    hold on;
    plot(1:size(arm1_joint_errors_deg,1), arm1_joint_errors_deg(:,2), 'g-', 'LineWidth', 1);
    plot(1:size(arm2_joint_errors_deg,1), arm2_joint_errors_deg(:,1), 'r--', 'LineWidth', 1);
    plot(1:size(arm2_joint_errors_deg,1), arm2_joint_errors_deg(:,2), 'g--', 'LineWidth', 1);
    xlabel('数据点');
    ylabel('关节角度误差 (度)');
    title('关节1和关节2误差对比');
    legend('机械臂1-关节1', '机械臂1-关节2', '机械臂2-关节1', '机械臂2-关节2');
    grid on;
    
    % 关节角度误差对比 - 关节3和关节4
    subplot(2, 3, 2);
    plot(1:size(arm1_joint_errors_deg,1), arm1_joint_errors_deg(:,3), 'b-', 'LineWidth', 1);
    hold on;
    plot(1:size(arm1_joint_errors_deg,1), arm1_joint_errors_deg(:,4), 'm-', 'LineWidth', 1);
    plot(1:size(arm2_joint_errors_deg,1), arm2_joint_errors_deg(:,3), 'b--', 'LineWidth', 1);
    plot(1:size(arm2_joint_errors_deg,1), arm2_joint_errors_deg(:,4), 'm--', 'LineWidth', 1);
    xlabel('数据点');
    ylabel('关节角度误差 (度)');
    title('关节3和关节4误差对比');
    legend('机械臂1-关节3', '机械臂1-关节4', '机械臂2-关节3', '机械臂2-关节4');
    grid on;
    
    % 位置误差对比 - X和Y轴
    subplot(2, 3, 3);
    plot(1:size(arm1_position_errors_m,1), arm1_position_errors_m(:,1)*1000, 'r-', 'LineWidth', 1);
    hold on;
    plot(1:size(arm1_position_errors_m,1), arm1_position_errors_m(:,2)*1000, 'g-', 'LineWidth', 1);
    plot(1:size(arm2_position_errors_m,1), arm2_position_errors_m(:,1)*1000, 'r--', 'LineWidth', 1);
    plot(1:size(arm2_position_errors_m,1), arm2_position_errors_m(:,2)*1000, 'g--', 'LineWidth', 1);
    xlabel('数据点');
    ylabel('位置误差 (mm)');
    title('X和Y轴位置误差对比');
    legend('机械臂1-X轴', '机械臂1-Y轴', '机械臂2-X轴', '机械臂2-Y轴');
    grid on;
    
    % 位置误差对比 - Z轴
    subplot(2, 3, 4);
    plot(1:size(arm1_position_errors_m,1), arm1_position_errors_m(:,3)*1000, 'b-', 'LineWidth', 1);
    hold on;
    plot(1:size(arm2_position_errors_m,1), arm2_position_errors_m(:,3)*1000, 'b--', 'LineWidth', 1);
    xlabel('数据点');
    ylabel('位置误差 (mm)');
    title('Z轴位置误差对比');
    legend('机械臂1-Z轴', '机械臂2-Z轴');
    grid on;
    
    % 总体位置误差对比
    subplot(2, 3, 5);
    plot(1:length(arm1_total_pos_error), arm1_total_pos_error*1000, 'k-', 'LineWidth', 1.5);
    hold on;
    plot(1:length(arm2_total_pos_error), arm2_total_pos_error*1000, 'k--', 'LineWidth', 1.5);
    xlabel('数据点');
    ylabel('总体位置误差 (mm)');
    title('总体位置误差对比');
    legend('机械臂1', '机械臂2');
    grid on;
    
    % 误差分布对比
    subplot(2, 3, 6);
    histogram(arm1_total_pos_error*1000, 30, 'FaceAlpha', 0.7, 'FaceColor', 'blue');
    hold on;
    histogram(arm2_total_pos_error*1000, 30, 'FaceAlpha', 0.7, 'FaceColor', 'red');
    xlabel('总体位置误差 (mm)');
    ylabel('频次');
    title('总体位置误差分布对比');
    legend('机械臂1', '机械臂2');
    grid on;
end