function dual_arm_gui()
    %% === 加载 URDF 模型 ===
    global DuArm q logFilePath label sliderLabels

    [DuArm, ~] = importrobot("YZYstructureWithoutInput", 'DataFormat', 'column');
    q = zeros(numel(homeConfiguration(DuArm)), 1);  % 初始化角度

    % 初始化 CSV 路径
    logFilePath = 'joint_log.csv';
    if isfile(logFilePath)
        delete(logFilePath); % 启动时清空旧文件
    end

    %% === 创建主界面 ===
    f = figure('Name', 'YZY URDF Real-time Control', ...
               'Position', [100, 100, 1200, 600]);

    ax = axes('Parent', f, 'Position', [0.05 0.3 0.5 0.65]);
    axis equal
    view(135, 25)

    % 显示末端位置
    label = uicontrol('Style', 'text', 'Units', 'normalized', ...
        'Position', [0.1, 0.05, 0.8, 0.05], 'FontSize', 12, ...
        'HorizontalAlignment', 'left', ...
        'String', 'End Effector Position: [x y z]');

    % 存储动态 label 句柄用于更新角度显示
    sliderLabels = gobjects(8, 1);

    % 显示初始姿态
    redraw(ax);

    %% === 创建滑动条控制每个关节 ===
    n = numel(q);
    for i = 1:n
        % 滑动条
        sld = uicontrol('Style', 'slider', ...
            'Min', -pi, 'Max', pi, 'Value', 0, ...
            'Units', 'normalized', ...
            'Position', [0.6, 0.92 - 0.08*i, 0.35, 0.05]);

        % 角度显示 label
        sliderLabels(i) = uicontrol('Style', 'text', ...
            'Units', 'normalized', ...
            'Position', [0.55, 0.92 - 0.08*i, 0.05, 0.05], ...
            'String', sprintf('J%d: %.2f', i, 0));

        addlistener(sld, 'ContinuousValueChange', ...
            @(src, ~) updateJoint(i, src.Value, ax));
    end
end

%% === 更新关节角 + 重绘 + 写入CSV ===
function updateJoint(index, value, ax)
    global q sliderLabels
    q(index) = value;

    % 更新角度文本
    sliderLabels(index).String = sprintf('J%d: %.2f', index, value);

    % 保存当前关节角和末端信息到 CSV
    saveJointAnglesToCSV();

    % 重绘图形
    redraw(ax);
end

%% === 重绘图像（保持相机角度） ===
function redraw(ax)
    global DuArm q label

    % === 记录当前相机状态 ===
    camPos = get(ax, 'CameraPosition');
    camTarget = get(ax, 'CameraTarget');
    camUpVec = get(ax, 'CameraUpVector');
    camViewAngle = get(ax, 'CameraViewAngle');

    % === 清除并重绘 ===
    cla(ax);
    show(DuArm, q, 'Visuals', 'on', 'PreservePlot', false, 'Parent', ax);

    % === 恢复相机状态 ===
    set(ax, 'CameraPosition', camPos);
    set(ax, 'CameraTarget', camTarget);
    set(ax, 'CameraUpVector', camUpVec);
    set(ax, 'CameraViewAngle', camViewAngle);

    % === 获取末端位姿 ===
    T4 = getTransform(DuArm, q, 'Body4');
    T8 = getTransform(DuArm, q, 'Body8');

    pos4 = tform2trvec(T4);
    pos8 = tform2trvec(T8);

    label.String = sprintf('Body4: [%.3f %.3f %.3f] | Body8: [%.3f %.3f %.3f]', ...
                            pos4, pos8);

    drawnow limitrate;
end

%% === 保存关节角 + 末端位姿到CSV ===
function saveJointAnglesToCSV()
    global q logFilePath DuArm

    % 获取 Body4 位姿
    T4 = getTransform(DuArm, q, 'Body4');
    p4 = tform2trvec(T4);
    r4 = rotm2eul(tform2rotm(T4));  % RPY

    % 获取 Body8 位姿
    T8 = getTransform(DuArm, q, 'Body8');
    p8 = tform2trvec(T8);
    r8 = rotm2eul(tform2rotm(T8));

    % 合并数据
    dataRow = [q', p4, r4, p8, r8];  % 共20个值

    % 写入表头（如第一次）
    if ~isfile(logFilePath)
        fid = fopen(logFilePath, 'w');
        fprintf(fid, 'J1,J2,J3,J4,J5,J6,J7,J8,');
        fprintf(fid, 'B4_px,B4_py,B4_pz,B4_r,B4_p,B4_y,');
        fprintf(fid, 'B8_px,B8_py,B8_pz,B8_r,B8_p,B8_y\n');
        fclose(fid);
    end

    % 追加数据
    dlmwrite(logFilePath, dataRow, '-append');
end
