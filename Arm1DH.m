% 机械臂1正运动学计算与误差分析程序（修正版）
% 根据DH表格计算末端执行器位置并与实际位置比较误差
% 修正：CSV中的关节角度从DH表第二行开始加

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

% 提取关节角度（前4列）和实际末端位置（第9-11列，只取x,y,z位置）
joint_angles_rad = table2array(data(:, 1:4));  % 前4个关节角度（弧度）
actual_positions = table2array(data(:, 9:11));  % 实际末端位置（x,y,z，单位：m）

% 机械臂1的DH参数表（基于初始角度）
% DH参数: [theta_initial, d, a, alpha] (角度为度，长度为mm)
% 第一行：从世界坐标系原点到关节1的转换（固定变换）
% 第二行及以后：从关节i到关节i+1的变换
DH_params_initial = [
    0,    109,   5.1,  -90;  % 世界坐标系到关节1的转换（固定）
    0,    5.3,   30.9,  90;  % 关节1到关节2的变换（关节1角度变化）
    0,    0,     270,  -90;  % 关节2到关节3的变换（关节2角度变化）
    90,   22.8,  0,     90;  % 关节3到关节4的变换（关节3角度变化）
    0,    265,   0,     0    % 关节4到关节5的变换（关节4角度变化）
];

% 初始化结果存储
num_points = size(joint_angles_rad, 1);
calculated_positions = zeros(num_points, 3);  % 计算得到的位置(x,y,z)，单位：m
position_errors = zeros(num_points, 3);       % 位置误差(x,y,z)，单位：m

fprintf('开始计算正运动学...\n');

% 对每一行数据进行正运动学计算
for i = 1:num_points
    % 获取当前关节角度（CSV中的角度，单位：弧度）
    current_joint_angles_rad = joint_angles_rad(i, :);
    
    % 计算正运动学
    T_final = forward_kinematics(current_joint_angles_rad, DH_params_initial);
    
    % 提取位置（mm转换为m）
    position_mm = T_final(1:3, 4);
    position_m = position_mm / 1000;  % mm转换为m
    
    % 存储计算结果
    calculated_positions(i, :) = position_m';
    
    % 计算位置误差（单位：m）
    position_errors(i, :) = position_m' - actual_positions(i, :);
    
    % 显示进度
    if mod(i, 100) == 0 || i == num_points
        fprintf('已处理 %d/%d 个数据点\n', i, num_points);
    end
end

% 计算统计数据
fprintf('\n=== 正运动学计算完成 ===\n');
fprintf('处理数据点数量: %d\n', num_points);

% 位置误差统计（单位：m）
pos_mean_error = mean(abs(position_errors), 1);
pos_max_error = max(abs(position_errors), [], 1);
pos_rms_error = sqrt(mean(position_errors.^2, 1));

fprintf('\n=== 位置误差统计 (m) ===\n');
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

% 总体位置误差（单位：m）
total_pos_error = sqrt(sum(position_errors.^2, 2));
fprintf('\n=== 总体位置误差统计 ===\n');
fprintf('平均总体误差: %.6f m (%.3f mm)\n', mean(total_pos_error), mean(total_pos_error)*1000);
fprintf('最大总体误差: %.6f m (%.3f mm)\n', max(total_pos_error), max(total_pos_error)*1000);
fprintf('RMS总体误差: %.6f m (%.3f mm)\n', sqrt(mean(total_pos_error.^2)), sqrt(mean(total_pos_error.^2))*1000);

% 保存结果到新的CSV文件
result_filename = 'kinematics_analysis_result.csv';
result_table = table(joint_angles_rad(:,1), joint_angles_rad(:,2), joint_angles_rad(:,3), joint_angles_rad(:,4), ...
                    calculated_positions(:,1), calculated_positions(:,2), calculated_positions(:,3), ...
                    actual_positions(:,1), actual_positions(:,2), actual_positions(:,3), ...
                    position_errors(:,1), position_errors(:,2), position_errors(:,3), ...
                    total_pos_error, ...
                    'VariableNames', {'Joint1_rad', 'Joint2_rad', 'Joint3_rad', 'Joint4_rad', ...
                                    'Calc_X_m', 'Calc_Y_m', 'Calc_Z_m', ...
                                    'Actual_X_m', 'Actual_Y_m', 'Actual_Z_m', ...
                                    'Error_X_m', 'Error_Y_m', 'Error_Z_m', ...
                                    'Total_Pos_Error_m'});

writetable(result_table, result_filename);
fprintf('\n结果已保存到文件: %s\n', result_filename);

% 绘制误差图表
create_error_plots(position_errors, total_pos_error);

fprintf('\n程序执行完成！\n');

%% 正运动学计算函数（修正版）
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
            % 使用DH表中的初始角度 + CSV中的关节角度
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

%% 绘制误差图表函数
function create_error_plots(position_errors, total_pos_error)
    % 创建误差分析图表
    
    figure('Name', '正运动学误差分析', 'Position', [100, 100, 1200, 600]);
    
    % 位置误差图（单位：m）
    subplot(2, 2, 1);
    plot(1:size(position_errors,1), position_errors(:,1), 'r-', 'LineWidth', 1);
    hold on;
    plot(1:size(position_errors,1), position_errors(:,2), 'g-', 'LineWidth', 1);
    plot(1:size(position_errors,1), position_errors(:,3), 'b-', 'LineWidth', 1);
    xlabel('数据点');
    ylabel('位置误差 (m)');
    title('位置误差对比');
    legend('X轴误差', 'Y轴误差', 'Z轴误差');
    grid on;
    
    % 位置误差图（单位：mm）
    subplot(2, 2, 2);
    plot(1:size(position_errors,1), position_errors(:,1)*1000, 'r-', 'LineWidth', 1);
    hold on;
    plot(1:size(position_errors,1), position_errors(:,2)*1000, 'g-', 'LineWidth', 1);
    plot(1:size(position_errors,1), position_errors(:,3)*1000, 'b-', 'LineWidth', 1);
    xlabel('数据点');
    ylabel('位置误差 (mm)');
    title('位置误差对比 (mm)');
    legend('X轴误差', 'Y轴误差', 'Z轴误差');
    grid on;
    
    % 总体位置误差图
    subplot(2, 2, 3);
    plot(1:length(total_pos_error), total_pos_error*1000, 'k-', 'LineWidth', 1.5);
    xlabel('数据点');
    ylabel('总体位置误差 (mm)');
    title('总体位置误差');
    grid on;
    
    % 误差分布直方图
    subplot(2, 2, 4);
    histogram(total_pos_error*1000, 50);
    xlabel('总体位置误差 (mm)');
    ylabel('频次');
    title('误差分布直方图');
    grid on;
    
    % 保存图像
    saveas(gcf, 'kinematics_error_analysis.png');
    fprintf('误差分析图表已保存为: kinematics_error_analysis.png\n');
end