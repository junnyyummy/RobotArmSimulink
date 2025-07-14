% 机械臂1逆运动学数值解法与误差分析程序
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
if width(data) < 11
    error('CSV文件列数不足，需要至少11列数据（前4列为关节角度，第9-11列为末端位置）');
end

% 提取数据
actual_joint_angles_rad = table2array(data(:, 1:4));    % 实际关节角度（弧度）
target_positions_m = table2array(data(:, 9:11));        % 目标末端位置（x,y,z，单位：m）

% 机械臂1的DH参数表
% DH参数: [theta_initial, d, a, alpha] (角度为度，长度为mm)
% 第一行：从世界坐标系原点到关节1的转换（固定变换）
% 第二行及以后：从关节i到关节i+1的变换
DH_params_initial = [
    0,    109,   5.1,  -90;  % 世界坐标系到关节1的转换（固定）
    0,    5.3,   30.9,  90;  % 关节1到关节2的变换
    0,    0,     270,  -90;  % 关节2到关节3的变换
    90,   22.8,  0,     90;  % 关节3到关节4的变换
    0,    250,   0,     0    % 关节4到关节5的变换
];

% 初始化结果存储
num_points = size(target_positions_m, 1);
solved_joint_angles_rad = zeros(num_points, 4);         % 求解的关节角度（弧度）
joint_angle_errors_rad = zeros(num_points, 4);          % 关节角度误差（弧度）
joint_angle_errors_deg = zeros(num_points, 4);          % 关节角度误差（度）
calculated_positions_m = zeros(num_points, 3);          % 用求解角度计算的位置（m）
position_errors_m = zeros(num_points, 3);               % 位置误差（m）
convergence_flags = zeros(num_points, 1);               % 收敛标志

% 逆运动学求解参数
options = optimset('Display', 'off', 'TolFun', 1e-6, 'TolX', 1e-6, 'MaxIter', 500);

fprintf('开始逆运动学求解...\n');

% 对每一个目标位置进行逆运动学求解
for i = 1:num_points
    target_pos_mm = target_positions_m(i, :) * 1000;  % 转换为mm
    
    % 使用实际关节角度作为初始猜测
    initial_guess = actual_joint_angles_rad(i, :);
    
    % 定义目标函数
    objective_function = @(joint_angles) inverse_kinematics_objective(joint_angles, target_pos_mm, DH_params_initial);
    
    % 求解逆运动学
    try
        [solved_angles, fval, exitflag] = fminsearch(objective_function, initial_guess, options);
        solved_joint_angles_rad(i, :) = solved_angles;
        convergence_flags(i) = (exitflag == 1 && fval < 1e-3);  % 收敛判断
    catch ME
        fprintf('数据点 %d 求解失败: %s\n', i, ME.message);
        solved_joint_angles_rad(i, :) = initial_guess;
        convergence_flags(i) = 0;
    end
    
    % 计算关节角度误差
    joint_angle_errors_rad(i, :) = solved_joint_angles_rad(i, :) - actual_joint_angles_rad(i, :);
    joint_angle_errors_deg(i, :) = joint_angle_errors_rad(i, :) * 180 / pi;
    
    % 用求解的关节角度计算正运动学
    T_final = forward_kinematics(solved_joint_angles_rad(i, :), DH_params_initial);
    calculated_pos_mm = T_final(1:3, 4);
    calculated_positions_m(i, :) = calculated_pos_mm' / 1000;  % 转换为m
    
    % 计算位置误差
    position_errors_m(i, :) = calculated_positions_m(i, :) - target_positions_m(i, :);
    
    % 显示进度
    if mod(i, 100) == 0 || i == num_points
        fprintf('已处理 %d/%d 个数据点\n', i, num_points);
    end
end

% 统计收敛情况
converged_count = sum(convergence_flags);
fprintf('\n=== 逆运动学求解完成 ===\n');
fprintf('处理数据点数量: %d\n', num_points);
fprintf('收敛数据点数量: %d (%.1f%%)\n', converged_count, 100*converged_count/num_points);

% 关节角度误差统计（弧度）
fprintf('\n=== 关节角度误差统计 (弧度) ===\n');
joint_mean_error_rad = mean(abs(joint_angle_errors_rad), 1);
joint_max_error_rad = max(abs(joint_angle_errors_rad), [], 1);
joint_rms_error_rad = sqrt(mean(joint_angle_errors_rad.^2, 1));

for j = 1:4
    fprintf('关节%d - 平均误差: %.6f, 最大误差: %.6f, RMS误差: %.6f\n', ...
            j, joint_mean_error_rad(j), joint_max_error_rad(j), joint_rms_error_rad(j));
end

% 关节角度误差统计（度）
fprintf('\n=== 关节角度误差统计 (度) ===\n');
joint_mean_error_deg = mean(abs(joint_angle_errors_deg), 1);
joint_max_error_deg = max(abs(joint_angle_errors_deg), [], 1);
joint_rms_error_deg = sqrt(mean(joint_angle_errors_deg.^2, 1));

for j = 1:4
    fprintf('关节%d - 平均误差: %.3f, 最大误差: %.3f, RMS误差: %.3f\n', ...
            j, joint_mean_error_deg(j), joint_max_error_deg(j), joint_rms_error_deg(j));
end

% 位置误差统计
fprintf('\n=== 位置误差统计 (m) ===\n');
pos_mean_error = mean(abs(position_errors_m), 1);
pos_max_error = max(abs(position_errors_m), [], 1);
pos_rms_error = sqrt(mean(position_errors_m.^2, 1));

fprintf('X轴 - 平均误差: %.6f, 最大误差: %.6f, RMS误差: %.6f\n', ...
        pos_mean_error(1), pos_max_error(1), pos_rms_error(1));
fprintf('Y轴 - 平均误差: %.6f, 最大误差: %.6f, RMS误差: %.6f\n', ...
        pos_mean_error(2), pos_max_error(2), pos_rms_error(2));
fprintf('Z轴 - 平均误差: %.6f, 最大误差: %.6f, RMS误差: %.6f\n', ...
        pos_mean_error(3), pos_max_error(3), pos_rms_error(3));

fprintf('\n=== 位置误差统计 (mm) ===\n');
fprintf('X轴 - 平均误差: %.3f, 最大误差: %.3f, RMS误差: %.3f\n', ...
        pos_mean_error(1)*1000, pos_max_error(1)*1000, pos_rms_error(1)*1000);
fprintf('Y轴 - 平均误差: %.3f, 最大误差: %.3f, RMS误差: %.3f\n', ...
        pos_mean_error(2)*1000, pos_max_error(2)*1000, pos_rms_error(2)*1000);
fprintf('Z轴 - 平均误差: %.3f, 最大误差: %.3f, RMS误差: %.3f\n', ...
        pos_mean_error(3)*1000, pos_max_error(3)*1000, pos_rms_error(3)*1000);

% 总体位置误差
total_pos_error = sqrt(sum(position_errors_m.^2, 2));
fprintf('\n=== 总体位置误差统计 ===\n');
fprintf('平均总体误差: %.6f m (%.3f mm)\n', mean(total_pos_error), mean(total_pos_error)*1000);
fprintf('最大总体误差: %.6f m (%.3f mm)\n', max(total_pos_error), max(total_pos_error)*1000);
fprintf('RMS总体误差: %.6f m (%.3f mm)\n', sqrt(mean(total_pos_error.^2)), sqrt(mean(total_pos_error.^2))*1000);

% 保存结果到CSV文件
result_filename = 'inverse_kinematics_analysis_result.csv';
result_table = table(actual_joint_angles_rad(:,1), actual_joint_angles_rad(:,2), ...
                    actual_joint_angles_rad(:,3), actual_joint_angles_rad(:,4), ...
                    solved_joint_angles_rad(:,1), solved_joint_angles_rad(:,2), ...
                    solved_joint_angles_rad(:,3), solved_joint_angles_rad(:,4), ...
                    joint_angle_errors_rad(:,1), joint_angle_errors_rad(:,2), ...
                    joint_angle_errors_rad(:,3), joint_angle_errors_rad(:,4), ...
                    joint_angle_errors_deg(:,1), joint_angle_errors_deg(:,2), ...
                    joint_angle_errors_deg(:,3), joint_angle_errors_deg(:,4), ...
                    target_positions_m(:,1), target_positions_m(:,2), target_positions_m(:,3), ...
                    calculated_positions_m(:,1), calculated_positions_m(:,2), calculated_positions_m(:,3), ...
                    position_errors_m(:,1), position_errors_m(:,2), position_errors_m(:,3), ...
                    total_pos_error, convergence_flags, ...
                    'VariableNames', {'Actual_J1_rad', 'Actual_J2_rad', 'Actual_J3_rad', 'Actual_J4_rad', ...
                                    'Solved_J1_rad', 'Solved_J2_rad', 'Solved_J3_rad', 'Solved_J4_rad', ...
                                    'JointErr_J1_rad', 'JointErr_J2_rad', 'JointErr_J3_rad', 'JointErr_J4_rad', ...
                                    'JointErr_J1_deg', 'JointErr_J2_deg', 'JointErr_J3_deg', 'JointErr_J4_deg', ...
                                    'Target_X_m', 'Target_Y_m', 'Target_Z_m', ...
                                    'Calc_X_m', 'Calc_Y_m', 'Calc_Z_m', ...
                                    'PosErr_X_m', 'PosErr_Y_m', 'PosErr_Z_m', ...
                                    'Total_Pos_Error_m', 'Converged'});

writetable(result_table, result_filename);
fprintf('\n结果已保存到文件: %s\n', result_filename);

% 绘制误差图表
create_inverse_kinematics_plots(joint_angle_errors_deg, position_errors_m, total_pos_error, convergence_flags);

fprintf('\n程序执行完成！\n');

%% 逆运动学目标函数
function error = inverse_kinematics_objective(joint_angles, target_position_mm, DH_params)
    % 目标函数：最小化末端位置误差
    % 输入: joint_angles - 关节角度（弧度）
    %      target_position_mm - 目标位置（mm）
    %      DH_params - DH参数表
    % 输出: error - 位置误差的平方和
    
    % 计算正运动学
    T_final = forward_kinematics(joint_angles, DH_params);
    
    % 提取计算得到的位置
    calculated_position_mm = T_final(1:3, 4);
    
    % 计算位置误差
    position_error = calculated_position_mm' - target_position_mm;
    
    % 返回误差的平方和
    error = sum(position_error.^2);
end

%% 正运动学计算函数
function T_final = forward_kinematics(joint_angles_rad, DH_params_initial)
    % 输入: joint_angles_rad - 4个关节的转动角度 (弧度)
    %      DH_params_initial - DH参数矩阵 (角度为度，长度为mm)
    % 输出: T_final - 最终变换矩阵 (位置单位为mm)
    
    % 初始化为单位矩阵
    T_final = eye(4);
    
    % 逐个关节计算变换矩阵
    for i = 1:size(DH_params_initial, 1)
        if i == 1
            % 第一行：从世界坐标系到关节1的转换（固定变换）
            theta_rad = DH_params_initial(i, 1) * pi / 180;
        else
            % 第二行及以后：从关节i-1到关节i的变换
            % 使用DH表中的初始角度 + 关节转动角度
            theta_initial_rad = DH_params_initial(i, 1) * pi / 180;
            joint_index = i - 1;  % 关节索引（第2行对应关节1）
            
            if joint_index <= length(joint_angles_rad)
                theta_rad = theta_initial_rad + joint_angles_rad(joint_index);
            else
                % 如果没有对应的关节角度，使用初始角度
                theta_rad = theta_initial_rad;
            end
        end
        
        d_mm = DH_params_initial(i, 2);
        a_mm = DH_params_initial(i, 3);
        alpha_rad = DH_params_initial(i, 4) * pi / 180;
        
        % 计算单个关节的变换矩阵
        T_i = dh_transform(theta_rad, d_mm, a_mm, alpha_rad);
        
        % 累积变换
        T_final = T_final * T_i;
    end
end

%% DH变换矩阵计算函数
function T = dh_transform(theta, d, a, alpha)
    % 根据DH参数计算变换矩阵
    % 输入: theta, d, a, alpha - DH参数
    % 输出: T - 4x4变换矩阵
    
    T = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
         sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
         0,           sin(alpha),             cos(alpha),            d;
         0,           0,                      0,                     1];
end

%% 绘制逆运动学分析图表
function create_inverse_kinematics_plots(joint_errors_deg, position_errors_m, total_pos_error, convergence_flags)
    % 创建逆运动学误差分析图表
    
    figure('Name', '逆运动学误差分析', 'Position', [100, 100, 1400, 800]);
    
    % 关节角度误差图（度）
    subplot(2, 3, 1);
    plot(1:size(joint_errors_deg,1), joint_errors_deg(:,1), 'r-', 'LineWidth', 1);
    hold on;
    plot(1:size(joint_errors_deg,1), joint_errors_deg(:,2), 'g-', 'LineWidth', 1);
    plot(1:size(joint_errors_deg,1), joint_errors_deg(:,3), 'b-', 'LineWidth', 1);
    plot(1:size(joint_errors_deg,1), joint_errors_deg(:,4), 'm-', 'LineWidth', 1);
    xlabel('数据点');
    ylabel('关节角度误差 (度)');
    title('关节角度误差对比');
    legend('关节1', '关节2', '关节3', '关节4');
    grid on;
    
    % 位置误差图（mm）
    subplot(2, 3, 2);
    plot(1:size(position_errors_m,1), position_errors_m(:,1)*1000, 'r-', 'LineWidth', 1);
    hold on;
    plot(1:size(position_errors_m,1), position_errors_m(:,2)*1000, 'g-', 'LineWidth', 1);
    plot(1:size(position_errors_m,1), position_errors_m(:,3)*1000, 'b-', 'LineWidth', 1);
    xlabel('数据点');
    ylabel('位置误差 (mm)');
    title('位置误差对比');
    legend('X轴误差', 'Y轴误差', 'Z轴误差');
    grid on;
    
    % 总体位置误差图
    subplot(2, 3, 3);
    plot(1:length(total_pos_error), total_pos_error*1000, 'k-', 'LineWidth', 1.5);
    xlabel('数据点');
    ylabel('总体位置误差 (mm)');
    title('总体位置误差');
    grid on;
    
    % 关节角度误差分布直方图
    subplot(2, 3, 4);
    joint_total_error = sqrt(sum(joint_errors_deg.^2, 2));
    histogram(joint_total_error, 50);
    xlabel('关节角度总误差 (度)');
    ylabel('频次');
    title('关节角度误差分布');
    grid on;
    
    % 位置误差分布直方图
    subplot(2, 3, 5);
    histogram(total_pos_error*1000, 50);
    xlabel('总体位置误差 (mm)');
    ylabel('频次');
    title('位置误差分布');
    grid on;
    
    % 收敛情况图
    subplot(2, 3, 6);
    plot(1:length(convergence_flags), convergence_flags, 'bo-', 'MarkerSize', 3);
    xlabel('数据点');
    ylabel('收敛标志');
    title('逆运动学收敛情况');
    ylim([-0.1, 1.1]);
    grid on;
    
    % 保存图像
    saveas(gcf, 'inverse_kinematics_analysis.png');
    fprintf('逆运动学分析图表已保存为: inverse_kinematics_analysis.png\n');
end