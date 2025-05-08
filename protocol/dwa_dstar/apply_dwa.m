function refined_path = apply_dwa(map, global_path, goal)
% 参数优化
max_speed = 1.5;           % 增大最大线速度
max_yaw_rate = 1.8;        % 增大最大角速度
max_accel = 0.6;           % 增大最大线加速度
max_delta_yaw_rate = 0.6;  % 增大最大角加速度
v_resolution = 0.3;        % 增大线速度分辨率
yaw_rate_resolution = 0.1; % 增大角速度分辨率
dt = 0.2;                  % 增大时间步长
predict_time = 1.8;        % 减小预测时间

% 其他参数保持不变
heading_weight = 1.0;
dist_weight = 1.0;
velocity_weight = 0.1;
obstacle_weight = 0.1;
robot_radius = 1.0;

% 初始化路径
refined_path = [];
refined_path = [refined_path; global_path(1,:)];
current_pos = global_path(1,:);
current_vel = [0, 0];
current_yaw = atan2(goal(2) - current_pos(2), goal(1) - current_pos(1));

% 减小目标点数量 - 每隔几个点选择一个作为目标点
waypoint_interval = 2;  % 每5个点取1个
waypoints = global_path(1:waypoint_interval:end, :);
if ~isequal(waypoints(end,:), global_path(end,:))
    waypoints = [waypoints; global_path(end,:)];  % 确保包含终点
end

waypoint_index = 2;
while waypoint_index <= size(waypoints, 1)
    target = waypoints(waypoint_index, :);

    % 计算到目标的距离
    dist_to_target = norm(target - current_pos);

    % 如果接近当前航点，移动到下一个
    if dist_to_target < 3.0  % 增大接近阈值
        waypoint_index = waypoint_index + 1;
        if waypoint_index > size(waypoints, 1)
            break;
        end
        target = waypoints(waypoint_index, :);
    end

    % 使用DWA计算最优速度
    [v, yaw_rate] = calc_control_input(current_pos, current_yaw, current_vel, target, map,...
        max_speed, max_yaw_rate, max_accel, max_delta_yaw_rate,...
        v_resolution, yaw_rate_resolution, ...
        predict_time, dt, heading_weight, dist_weight, ...
        velocity_weight, obstacle_weight, robot_radius);

    % 更新位置和方向
    current_pos = current_pos + [v * cos(current_yaw) * dt, v * sin(current_yaw) * dt];
    current_yaw = current_yaw + yaw_rate * dt;
    current_vel = [v, yaw_rate];

    % 添加新位置到细化路径
    refined_path = [refined_path; round(current_pos)];

    % 检查是否到达目标
    if norm(current_pos - goal) < 3.0  % 增大到达阈值
        break;
    end
end

% 确保目标包含在路径中
if norm(refined_path(end, :) - goal) > 1.0
    refined_path = [refined_path; goal];
end

% 删除重复点、检查对角线通过并平滑路径
refined_path = process_path_with_diagonal_check(refined_path, map);
end

function [optimal_v, optimal_yaw_rate] = calc_control_input(x, yaw, vel, target, map,...
    max_speed, max_yaw_rate, ...
    max_accel, max_delta_yaw_rate,...
    v_res, yaw_rate_res, ...
    predict_time, dt, ...
    heading_weight, dist_weight, ...
    velocity_weight, obstacle_weight, ...
    robot_radius)
% 计算动态窗口
Vs = calc_dynamic_window(vel, [max_speed, max_yaw_rate, max_accel, max_delta_yaw_rate]);

% 减小搜索步长，提高效率
v_res = 0.3;  % 增大速度分辨率
yaw_rate_res = 0.2;  % 增大角速度分辨率

% 减小预测时间和步长，加速计算
predict_time = 1.8;  % 减小预测时间

% 评估动态窗口中的所有速度
best_score = -Inf;
optimal_v = 0;
optimal_yaw_rate = 0;

for v = Vs(1):v_res:Vs(2)
    for yaw_rate = Vs(3):yaw_rate_res:Vs(4)
        % 使用向量化操作预测轨迹
        trajectory = predict_trajectory(x, yaw, v, yaw_rate, predict_time, dt);

        if isempty(trajectory)
            continue;
        end

        % 快速障碍检测 - 如果轨迹点位于障碍物上，直接跳过
        valid_trajectory = true;
        for i = 1:size(trajectory, 1)
            tx = round(trajectory(i, 1));
            ty = round(trajectory(i, 2));
            if tx < 1 || tx > size(map, 1) || ty < 1 || ty > size(map, 2) || map(tx, ty) >= 1
                valid_trajectory = false;
                break;
            end
        end

        if ~valid_trajectory
            continue;
        end

        % 检查轨迹是否尝试通过对角障碍物
        if ~check_trajectory_diagonal_validity(trajectory, map)
            continue;
        end

        % 计算得分
        heading_score = calc_heading_score(trajectory, target);
        dist_score = calc_dist_score(trajectory, target);
        vel_score = calc_velocity_score(v, max_speed);
        obstacle_score = calc_obstacle_score(trajectory, map, robot_radius);

        % 最终得分（各个得分的加权和）
        final_score = heading_weight * heading_score + ...
            dist_weight * dist_score + ...
            velocity_weight * vel_score + ...
            obstacle_weight * obstacle_score;

        % 更新最佳得分
        if final_score > best_score
            best_score = final_score;
            optimal_v = v;
            optimal_yaw_rate = yaw_rate;
        end
    end
end
end

function dw = calc_dynamic_window(vel, config)
% 计算基于当前速度和机器人配置的动态窗口
% config = [max_speed, max_yaw_rate, max_accel, max_delta_yaw_rate]

% 机器人动力学约束
Vs = [0, config(1), -config(2), config(2)]; % [min_v, max_v, min_yaw_rate, max_yaw_rate]

% 基于当前速度的动态约束
Vd = [vel(1) - config(3), vel(1) + config(3), vel(2) - config(4), vel(2) + config(4)];

% 最终动态窗口
dw = [max(Vs(1), Vd(1)), min(Vs(2), Vd(2)), max(Vs(3), Vd(3)), min(Vs(4), Vd(4))];
end

function trajectory = predict_trajectory(x, yaw, v, yaw_rate, predict_time, dt)
% Predict the trajectory for a given velocity
trajectory = zeros(floor(predict_time / dt), 2);
time = 0;
curr_x = x;
curr_yaw = yaw;

for i = 1:length(trajectory)
    time = time + dt;
    curr_x = curr_x + v * [cos(curr_yaw), sin(curr_yaw)] * dt;
    curr_yaw = curr_yaw + yaw_rate * dt;
    trajectory(i, :) = curr_x;
end
end

function score = calc_heading_score(trajectory, goal)
% Calculate heading score: alignment with goal direction
if isempty(trajectory)
    score = 0;
    return;
end

last_pos = trajectory(end, :);
goal_direction = atan2(goal(2) - last_pos(2), goal(1) - last_pos(1));
dx = goal(1) - last_pos(1);
dy = goal(2) - last_pos(2);
alignment = abs(atan2(dy, dx) - goal_direction);
score = pi - min(alignment, 2*pi - alignment);
score = score / pi;  % Normalize to [0, 1]
end

function score = calc_dist_score(trajectory, goal)
% Calculate distance score: distance to goal
if isempty(trajectory)
    score = 0;
    return;
end

dist = norm(trajectory(end, :) - goal);
score = 1.0 / (1.0 + dist);  % Closer to goal = higher score
end

function score = calc_velocity_score(v, max_v)
% Calculate velocity score: prefer higher speeds
score = v / max_v;  % Normalize to [0, 1]
end

function score = calc_obstacle_score(trajectory, map, robot_radius)
% 计算障碍物得分：惩罚接近障碍物
if isempty(trajectory)
    score = 0;
    return;
end

% 减小检查半径，只检查机器人半径范围内
check_radius = ceil(robot_radius);

% 只检查部分轨迹点，例如每3个点检查一次
check_interval = 3;
check_indices = 1:check_interval:size(trajectory, 1);

for i = check_indices
    x = round(trajectory(i, 1));
    y = round(trajectory(i, 2));

    % 跳过地图外的点
    if x < 1 || x > size(map, 1) || y < 1 || y > size(map, 2)
        score = 0;
        return;
    end

    % 检查是否位于障碍物上
    if map(x, y) >= 1
        score = 0;
        return;
    end

    % 快速检查障碍物距离 - 只检查机器人半径范围内
    for dx = -check_radius:check_radius
        for dy = -check_radius:check_radius
            nx = x + dx;
            ny = y + dy;

            % 跳过地图外的点
            if nx < 1 || nx > size(map, 1) || ny < 1 || ny > size(map, 2)
                continue;
            end

            if map(nx, ny) >= 1
                dist = sqrt(dx^2 + dy^2);
                if dist <= robot_radius
                    score = 0;  % 太接近障碍物
                    return;
                end
            end
        end
    end
end

% 如果所有点都通过检查，则安全
score = 1.0;
end

function valid = check_trajectory_diagonal_validity(trajectory, map)
% 检查轨迹是否尝试通过对角障碍物
if size(trajectory, 1) < 2
    valid = true;
    return;
end

% 对轨迹点进行离散化
discrete_traj = round(trajectory);

% 检查相邻点之间是否存在对角线通过
for i = 2:size(discrete_traj, 1)
    curr_x = discrete_traj(i, 1);
    curr_y = discrete_traj(i, 2);
    prev_x = discrete_traj(i-1, 1);
    prev_y = discrete_traj(i-1, 2);

    % 检查是否为对角线移动
    if abs(curr_x - prev_x) == 1 && abs(curr_y - prev_y) == 1
        % 检查对角线移动是否通过障碍物
        if curr_x < 1 || curr_x > size(map, 1) || curr_y < 1 || curr_y > size(map, 2) || ...
                prev_x < 1 || prev_x > size(map, 1) || prev_y < 1 || prev_y > size(map, 2)
            % 超出地图边界
            valid = false;
            return;
        end

        % 检查两个对角相邻格子是否都是障碍物
        if map(prev_x, curr_y) >= 1 && map(curr_x, prev_y) >= 1
            valid = false;
            return;
        end
    end

    % 对于较长的移动，插值检查中间点
    dx = curr_x - prev_x;
    dy = curr_y - prev_y;
    if abs(dx) > 1 || abs(dy) > 1
        steps = max(abs(dx), abs(dy));
        for s = 1:steps-1
            inter_x = round(prev_x + dx * s / steps);
            inter_y = round(prev_y + dy * s / steps);
            next_x = round(prev_x + dx * (s+1) / steps);
            next_y = round(prev_y + dy * (s+1) / steps);

            % 检查插值点的对角线移动
            if abs(next_x - inter_x) == 1 && abs(next_y - inter_y) == 1
                if inter_x < 1 || inter_x > size(map, 1) || inter_y < 1 || inter_y > size(map, 2) || ...
                        next_x < 1 || next_x > size(map, 1) || next_y < 1 || next_y > size(map, 2)
                    valid = false;
                    return;
                end

                % 检查对角相邻格子是否都是障碍物
                if map(inter_x, next_y) >= 1 && map(next_x, inter_y) >= 1
                    valid = false;
                    return;
                end
            end
        end
    end
end

valid = true;
end

function path = process_path_with_diagonal_check(path, map)
% 处理路径：移除重复点，检查对角线有效性，平滑路径
if size(path, 1) <= 1
    return;
end

% 移除重复点
unique_path = path(1,:);
for i = 2:size(path, 1)
    if ~isequal(path(i,:), path(i-1,:))
        unique_path = [unique_path; path(i,:)];
    end
end

% 检查路径中的每个对角线移动并修复无效的对角线通过
fixed_path = unique_path(1,:);
for i = 2:size(unique_path, 1)
    curr_x = unique_path(i, 1);
    curr_y = unique_path(i, 2);
    prev_x = unique_path(i-1, 1);
    prev_y = unique_path(i-1, 2);

    % 检查是否为对角线移动
    if abs(curr_x - prev_x) == 1 && abs(curr_y - prev_y) == 1
        % 检查对角线是否有效
        if curr_x >= 1 && curr_x <= size(map, 1) && curr_y >= 1 && curr_y <= size(map, 2) && ...
                prev_x >= 1 && prev_x <= size(map, 1) && prev_y >= 1 && prev_y <= size(map, 2)

            % 如果两个对角相邻格子都是障碍物，添加一个中间点以避免对角线通过
            if map(prev_x, curr_y) >= 1 && map(curr_x, prev_y) >= 1
                % 尝试两种可能的绕行路径
                if prev_x < curr_x
                    if prev_y < curr_y
                        % 从左下到右上
                        if map(prev_x, curr_y) < map(curr_x, prev_y)
                            % 使用左上路径
                            fixed_path = [fixed_path; prev_x, curr_y];
                        else
                            % 使用右下路径
                            fixed_path = [fixed_path; curr_x, prev_y];
                        end
                    else
                        % 从左上到右下
                        if map(prev_x, curr_y) < map(curr_x, prev_y)
                            % 使用左下路径
                            fixed_path = [fixed_path; prev_x, curr_y];
                        else
                            % 使用右上路径
                            fixed_path = [fixed_path; curr_x, prev_y];
                        end
                    end
                else
                    if prev_y < curr_y
                        % 从右下到左上
                        if map(prev_x, curr_y) < map(curr_x, prev_y)
                            % 使用右上路径
                            fixed_path = [fixed_path; prev_x, curr_y];
                        else
                            % 使用左下路径
                            fixed_path = [fixed_path; curr_x, prev_y];
                        end
                    else
                        % 从右上到左下
                        if map(prev_x, curr_y) < map(curr_x, prev_y)
                            % 使用右下路径
                            fixed_path = [fixed_path; prev_x, curr_y];
                        else
                            % 使用左上路径
                            fixed_path = [fixed_path; curr_x, prev_y];
                        end
                    end
                end
            end
        end
    end

    % 添加当前点到修复后的路径
    fixed_path = [fixed_path; curr_x, curr_y];
end

% 简单路径平滑 - 使用移动平均滤波
if size(fixed_path, 1) <= 2
    path = fixed_path;
    return;
end

smoothed_path = fixed_path;
window_size = 3;
for i = 2:(size(fixed_path, 1)-1)
    start_idx = max(1, i-floor(window_size/2));
    end_idx = min(size(fixed_path, 1), i+floor(window_size/2));
    window = fixed_path(start_idx:end_idx, :);
    smoothed_path(i,:) = round(mean(window, 1));
end

% 最终路径经过对角线检查
final_path = verify_path_diagonal_safety(smoothed_path, map);

path = final_path;
end

function safe_path = verify_path_diagonal_safety(path, map)
% 最终验证路径是否有对角线通过障碍物的问题，并进行修复
if size(path, 1) <= 1
    safe_path = path;
    return;
end

safe_path = path(1,:);
for i = 2:size(path, 1)
    curr_x = path(i, 1);
    curr_y = path(i, 2);
    prev_x = path(i-1, 1);
    prev_y = path(i-1, 2);

    % 检查对角线移动
    if abs(curr_x - prev_x) == 1 && abs(curr_y - prev_y) == 1
        % 检查两个对角相邻格子是否都是障碍物
        if curr_x >= 1 && curr_x <= size(map, 1) && curr_y >= 1 && curr_y <= size(map, 2) && ...
                prev_x >= 1 && prev_x <= size(map, 1) && prev_y >= 1 && prev_y <= size(map, 2) && ...
                map(prev_x, curr_y) >= 1 && map(curr_x, prev_y) >= 1

            % 选择障碍物值较小的路径
            if map(prev_x, curr_y) < map(curr_x, prev_y)
                safe_path = [safe_path; prev_x, curr_y];
            else
                safe_path = [safe_path; curr_x, prev_y];
            end
        end
    end

    safe_path = [safe_path; curr_x, curr_y];
end
end
