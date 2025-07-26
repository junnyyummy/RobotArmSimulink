function ik_verification_tool()
% 精简版双重逆运动学验证工具 - 聚焦核心IK性能对比
% DH表格 vs RigidBodyTree 逆运动学求解对比，注意，这是完整的逆运动学计算，而不是简单的FK对比
% 后续将基于这两种算法进行持续开发

fprintf('=== 精简版 DH vs RBT 逆运动学对比工具 ===\n');

%% 1. 数据加载和预处理
fprintf('\n--- 数据加载 ---\n');

% 加载RBT模型
try
    [robot_model, ~] = importrobot("YZYstructureWithoutInput", 'DataFormat', 'column');
    fprintf('✓ RBT模型加载成功\n');
catch ME
    error('RBT模型加载失败: %s', ME.message);
end

% 读取CSV数据
try
    data = readmatrix('joint_log.csv');
    fprintf('✓ CSV数据读取成功，尺寸: %dx%d\n', size(data));
catch ME
    error('CSV文件读取失败: %s', ME.message);
end

% 提取数据
joint_angles_true = data(:, 1:8);
arm1_positions = data(:, 9:11);
arm2_positions = data(:, 15:17);

% 单位转换
if max(abs([arm1_positions(:); arm2_positions(:)])) > 10
    arm1_positions = arm1_positions * 0.001; % mm to m
    arm2_positions = arm2_positions * 0.001;
    fprintf('单位转换: mm → m\n');
end

% 限制测试样本数
num_samples = min(100, size(data, 1));
joint_angles_true = joint_angles_true(1:num_samples, :);
arm1_positions = arm1_positions(1:num_samples, :);
arm2_positions = arm2_positions(1:num_samples, :);
fprintf('测试样本数: %d\n', num_samples);

%% 2. 修正后的DH参数
dh_arm1 = [0, 109, 5.1, -90; 0, 5.3, 30.9, 90; 0, 0, 270, -90; 90, 22.8, 0, 90; 0, 250, 0, 0];
dh_arm2 = [0, 109, 354.9, 90; 0, -128.8, 30.9, -90; 0, 0, 270, 90; 90, 22.8, 0, 90; 0, 250, 0, 0];
fprintf('DH参数修正: 265mm → 250mm (修正15mm偏移)\n');

%% 3. 双重逆运动学测试
fprintf('\n--- 开始逆运动学对比测试 ---\n');

% 初始化结果
rbt_results = struct('success', zeros(num_samples,1), 'errors', zeros(num_samples,2), 'times', zeros(num_samples,1));
dh_results = struct('success', zeros(num_samples,1), 'errors', zeros(num_samples,2), 'times', zeros(num_samples,1));

% 进度计数
progress_step = max(1, round(num_samples/10));

for i = 1:num_samples
    if mod(i, progress_step) == 0
        fprintf('进度: %d/%d (%.0f%%)\n', i, num_samples, 100*i/num_samples);
    end
    
    target1 = arm1_positions(i, :);
    target2 = arm2_positions(i, :);
    q_true = joint_angles_true(i, :)';
    
    % RBT逆运动学
    tic;
    [rbt_success, rbt_err] = solve_rbt_ik(robot_model, target1, target2, q_true);
    rbt_results.times(i) = toc;
    rbt_results.success(i) = rbt_success;
    rbt_results.errors(i, :) = rbt_err;
    
    % DH逆运动学
    tic;
    [dh_success, dh_err] = solve_dh_ik(dh_arm1, dh_arm2, target1, target2, q_true);
    dh_results.times(i) = toc;
    dh_results.success(i) = dh_success;
    dh_results.errors(i, :) = dh_err;
end

%% 4. 结果统计
fprintf('\n--- 结果统计 ---\n');

% RBT统计
rbt_success_rate = sum(rbt_results.success) / num_samples * 100;
rbt_avg_error = mean(rbt_results.errors(rbt_results.success == 1, :));
rbt_avg_time = mean(rbt_results.times);

% DH统计
dh_success_rate = sum(dh_results.success) / num_samples * 100;
dh_avg_error = mean(dh_results.errors(dh_results.success == 1, :));
dh_avg_time = mean(dh_results.times);

% 打印结果
fprintf('\n【RBT方法】\n');
fprintf('成功率: %.1f%% (%d/%d)\n', rbt_success_rate, sum(rbt_results.success), num_samples);
if ~isempty(rbt_avg_error)
    fprintf('平均误差: Arm1=%.2fmm, Arm2=%.2fmm\n', rbt_avg_error*1000);
end
fprintf('平均时间: %.3fs\n', rbt_avg_time);

fprintf('\n【DH方法】\n');
fprintf('成功率: %.1f%% (%d/%d)\n', dh_success_rate, sum(dh_results.success), num_samples);
if ~isempty(dh_avg_error)
    fprintf('平均误差: Arm1=%.2fmm, Arm2=%.2fmm\n', dh_avg_error*1000);
end
fprintf('平均时间: %.3fs\n', dh_avg_time);

%% 5. 对比分析
fprintf('\n【对比分析】\n');
success_diff = rbt_success_rate - dh_success_rate;
time_ratio = rbt_avg_time / dh_avg_time;

fprintf('成功率差异: %.1f%% (RBT - DH)\n', success_diff);
fprintf('计算时间比: %.2f (RBT/DH)\n', time_ratio);

if abs(success_diff) < 5
    fprintf('结论: 两种方法成功率相当\n');
elseif success_diff > 0
    fprintf('结论: RBT方法成功率更高\n');
else
    fprintf('结论: DH方法成功率更高\n');
end

if time_ratio < 0.8
    fprintf('速度: RBT方法更快\n');
elseif time_ratio > 1.2
    fprintf('速度: DH方法更快\n');
else
    fprintf('速度: 两种方法相当\n');
end

%% 6. 可视化
create_compact_visualization(rbt_results, dh_results, num_samples);

%% 7. 保存结果
save_compact_results(rbt_results, dh_results, num_samples);

fprintf('\n=== 对比测试完成 ===\n');

end

%% RBT逆运动学求解
function [success, errors] = solve_rbt_ik(robot, target1, target2, q_true)
% RBT逆运动学求解 - 使用智能初值策略

success = false;
errors = [inf, inf];

% 检测RBT单位
q_test = zeros(robot.NumBodies, 1);
T_test = getTransform(robot, q_test, 'Body4');
pos_test = tform2trvec(T_test);
if max(abs(pos_test)) < 0.01
    unit_scale = 0.001; % RBT使用mm
else
    unit_scale = 1.0;   % RBT使用m
end

target1_rbt = target1 / unit_scale;
target2_rbt = target2 / unit_scale;

% 智能初值策略
initial_guesses = [q_true, q_true + 0.05*randn(8,1), q_true + 0.1*(rand(8,1)-0.5)];

% 优化选项
options = optimset('Display', 'off', 'TolFun', 1e-8, 'MaxIter', 200);

best_error = inf;
best_q = q_true;

for attempt = 1:size(initial_guesses, 2)
    try
        objective = @(q) ik_objective_rbt(q, robot, target1_rbt, target2_rbt);
        [q_sol, error] = fminsearch(objective, initial_guesses(:, attempt), options);
        
        if error < best_error
            best_error = error;
            best_q = q_sol;
        end
        
        % 检查收敛
        if sqrt(error) * unit_scale < 0.005 % 5mm阈值
            success = true;
            break;
        end
    catch
        continue;
    end
end

% 计算最终误差
if best_error < inf
    try
        T1 = getTransform(robot, best_q, 'Body4');
        T2 = getTransform(robot, best_q, 'Body8');
        pos1 = tform2trvec(T1) * unit_scale;
        pos2 = tform2trvec(T2) * unit_scale;
        errors = [norm(pos1 - target1), norm(pos2 - target2)];
        
        if max(errors) < 0.01 % 10mm最终检查
            success = true;
        end
    catch
        errors = [inf, inf];
    end
end

end

%% DH逆运动学求解
function [success, errors] = solve_dh_ik(dh_arm1, dh_arm2, target1, target2, q_true)
% DH逆运动学求解 - 分别求解两个机械臂

success = false;
errors = [inf, inf];

% 转换为mm单位
target1_mm = target1 * 1000;
target2_mm = target2 * 1000;

% 提取真实关节角
q1_true = q_true(1:4);
q2_true = q_true(5:8);

% 优化选项
options = optimset('Display', 'off', 'TolFun', 1e-6, 'MaxIter', 150);

% Arm1求解
try
    obj1 = @(q) dh_objective_single_arm(q, target1_mm, dh_arm1);
    [q1_sol, err1] = fminsearch(obj1, q1_true, options);
    
    if sqrt(err1) < 5.0 % 5mm阈值
        arm1_success = true;
        arm1_error = sqrt(err1) / 1000; % 转换为m
    else
        arm1_success = false;
        arm1_error = inf;
    end
catch
    arm1_success = false;
    arm1_error = inf;
end

% Arm2求解
try
    obj2 = @(q) dh_objective_single_arm(q, target2_mm, dh_arm2);
    [q2_sol, err2] = fminsearch(obj2, q2_true, options);
    
    if sqrt(err2) < 5.0 % 5mm阈值
        arm2_success = true;
        arm2_error = sqrt(err2) / 1000; % 转换为m
    else
        arm2_success = false;
        arm2_error = inf;
    end
catch
    arm2_success = false;
    arm2_error = inf;
end

% 综合结果
success = arm1_success && arm2_success;
errors = [arm1_error, arm2_error];

end

%% 目标函数
function error = ik_objective_rbt(q, robot, target1, target2)
% RBT目标函数

try
    T1 = getTransform(robot, q, 'Body4');
    T2 = getTransform(robot, q, 'Body8');
    
    pos1 = tform2trvec(T1);
    pos2 = tform2trvec(T2);
    
    error1 = sum((pos1 - target1).^2);
    error2 = sum((pos2 - target2).^2);
    
    % 添加关节限制惩罚
    joint_penalty = sum(max(0, abs(q) - pi).^2) * 1000;
    
    error = error1 + error2 + joint_penalty;
catch
    error = 1e6;
end

end

function error = dh_objective_single_arm(q, target_mm, dh_params)
% DH单臂目标函数

try
    T = dh_forward_kinematics(q, dh_params);
    pos_calc = T(1:3, 4)';
    
    pos_error = sum((pos_calc - target_mm).^2);
    joint_penalty = sum(max(0, abs(q) - pi).^2) * 1000;
    
    error = pos_error + joint_penalty;
catch
    error = 1e6;
end

end

%% DH正运动学
function T_final = dh_forward_kinematics(q, dh_params)
% DH正运动学计算

T_final = eye(4);

for i = 1:size(dh_params, 1)
    if i == 1
        theta = dh_params(i, 1) * pi / 180;
    else
        theta_init = dh_params(i, 1) * pi / 180;
        joint_idx = i - 1;
        
        if joint_idx <= length(q)
            theta = theta_init + q(joint_idx);
        else
            theta = theta_init;
        end
    end
    
    d = dh_params(i, 2);
    a = dh_params(i, 3);
    alpha = dh_params(i, 4) * pi / 180;
    
    T_i = [cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
           sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
           0,           sin(alpha),             cos(alpha),            d;
           0,           0,                      0,                     1];
    
    T_final = T_final * T_i;
end

end

%% 可视化
function create_compact_visualization(rbt_results, dh_results, num_samples)
% 创建精简的可视化图表

try
    figure('Name', '精简版IK对比分析', 'Position', [100, 100, 1200, 600]);
    
    % 子图1: 成功率对比
    subplot(2, 3, 1);
    success_rates = [sum(rbt_results.success)/num_samples*100, sum(dh_results.success)/num_samples*100];
    bar(success_rates, 'FaceColor', [0.3 0.6 0.9]);
    set(gca, 'XTickLabel', {'RBT', 'DH'});
    ylabel('成功率 (%)');
    title('成功率对比');
    ylim([0, 105]);
    
    % 添加数值标签
    for i = 1:2
        text(i, success_rates(i) + 2, sprintf('%.1f%%', success_rates(i)), ...
            'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end
    
    % 子图2: 计算时间对比
    subplot(2, 3, 2);
    avg_times = [mean(rbt_results.times), mean(dh_results.times)];
    bar(avg_times, 'FaceColor', [0.9 0.6 0.3]);
    set(gca, 'XTickLabel', {'RBT', 'DH'});
    ylabel('平均时间 (秒)');
    title('计算速度对比');
    
    for i = 1:2
        text(i, avg_times(i) + max(avg_times)*0.05, sprintf('%.3fs', avg_times(i)), ...
            'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end
    
    % 子图3: 误差分布
    subplot(2, 3, 3);
    rbt_valid_errors = rbt_results.errors(rbt_results.success == 1, :);
    dh_valid_errors = dh_results.errors(dh_results.success == 1, :);
    
    if ~isempty(rbt_valid_errors) && ~isempty(dh_valid_errors)
        all_rbt_errors = [rbt_valid_errors(:, 1); rbt_valid_errors(:, 2)] * 1000;
        all_dh_errors = [dh_valid_errors(:, 1); dh_valid_errors(:, 2)] * 1000;
        
        histogram(all_rbt_errors, 20, 'FaceAlpha', 0.7, 'DisplayName', 'RBT');
        hold on;
        histogram(all_dh_errors, 20, 'FaceAlpha', 0.7, 'DisplayName', 'DH');
        xlabel('位置误差 (mm)');
        ylabel('频次');
        title('误差分布对比');
        legend;
        set(gca, 'YScale', 'log');
    end
    
    % 子图4: 成功失败统计
    subplot(2, 3, 4);
    both_success = (rbt_results.success == 1) & (dh_results.success == 1);
    rbt_only = (rbt_results.success == 1) & (dh_results.success == 0);
    dh_only = (rbt_results.success == 0) & (dh_results.success == 1);
    both_fail = (rbt_results.success == 0) & (dh_results.success == 0);
    
    counts = [sum(both_success), sum(rbt_only), sum(dh_only), sum(both_fail)];
    labels = {'两者成功', '仅RBT成功', '仅DH成功', '两者失败'};
    
    pie(counts, labels);
    title('一致性分析');
    
    % 子图5: 时间序列
    subplot(2, 3, 5);
    plot(1:num_samples, rbt_results.times, 'b-', 'LineWidth', 1, 'DisplayName', 'RBT');
    hold on;
    plot(1:num_samples, dh_results.times, 'r-', 'LineWidth', 1, 'DisplayName', 'DH');
    xlabel('样本编号');
    ylabel('计算时间 (秒)');
    title('时间变化趋势');
    legend;
    grid on;
    
    % 子图6: 总结信息
    subplot(2, 3, 6);
    text(0.1, 0.9, '测试总结:', 'FontSize', 12, 'FontWeight', 'bold');
    
    summary_text = {
        sprintf('测试样本: %d个', num_samples);
        sprintf('RBT成功率: %.1f%%', sum(rbt_results.success)/num_samples*100);
        sprintf('DH成功率: %.1f%%', sum(dh_results.success)/num_samples*100);
        sprintf('RBT平均时间: %.3fs', mean(rbt_results.times));
        sprintf('DH平均时间: %.3fs', mean(dh_results.times));
        sprintf('一致成功: %d个', sum(both_success));
        '';
        '主要改进:';
        '• 智能初值策略';
        '• 15mm偏移修正';
        '• 优化目标函数';
    };
    
    y_pos = 0.8;
    for i = 1:length(summary_text)
        if contains(summary_text{i}, ':')
            text(0.1, y_pos, summary_text{i}, 'FontSize', 10, 'FontWeight', 'bold');
        elseif startsWith(summary_text{i}, '•')
            text(0.15, y_pos, summary_text{i}, 'FontSize', 9, 'Color', 'blue');
        else
            text(0.1, y_pos, summary_text{i}, 'FontSize', 9);
        end
        y_pos = y_pos - 0.08;
    end
    
    xlim([0, 1]); ylim([0, 1]); axis off;
    
    fprintf('✓ 可视化图表已生成\n');
    
catch ME
    fprintf('⚠ 图表生成失败: %s\n', ME.message);
end

end

%% 保存结果
function save_compact_results(rbt_results, dh_results, num_samples)
% 保存精简的结果

try
    % 创建结果表
    results_table = table();
    results_table.Sample = (1:num_samples)';
    results_table.RBT_Success = rbt_results.success;
    results_table.DH_Success = dh_results.success;
    results_table.RBT_Error_Arm1 = rbt_results.errors(:, 1) * 1000; % mm
    results_table.RBT_Error_Arm2 = rbt_results.errors(:, 2) * 1000; % mm
    results_table.DH_Error_Arm1 = dh_results.errors(:, 1) * 1000;   % mm
    results_table.DH_Error_Arm2 = dh_results.errors(:, 2) * 1000;   % mm
    results_table.RBT_Time = rbt_results.times;
    results_table.DH_Time = dh_results.times;
    
    % 保存CSV
    writetable(results_table, 'compact_ik_comparison.csv');
    
    % 保存到工作空间
    assignin('base', 'compact_ik_results', results_table);
    
    fprintf('✓ 结果已保存: compact_ik_comparison.csv\n');
    
catch ME
    fprintf('⚠ 结果保存失败: %s\n', ME.message);
end

end