function [start, goal] = generate_start_goal(map)
% 生成符合约束条件的随机起点和终点
% 输入参数：
%   map - 二维矩阵地图，0表示自由空间，非0表示障碍物
% 输出参数：
%   start - 起点坐标 [row, col]
%   goal  - 终点坐标 [row, col]

%% 参数初始化
[height, width] = size(map);
map_center = [height/2, width/2]; % 几何中心坐标
min_center_dist = mean([height, width])/2; % 中心最小距离阈值
min_start_goal_dist = mean([height, width]); % 起终最小距离阈值
max_attempts = 1000; % 最大尝试次数

%% 起点生成
start = [];
attempt = 0;
while isempty(start) && attempt < max_attempts
    % 生成随机候选点
    candidate = [randi(height), randi(width)];

    % 障碍物检测
    if map(candidate(1), candidate(2)) ~= 0
        attempt = attempt + 1;
        continue;
    end

    % 计算到中心距离
    dist_to_center = norm(candidate - map_center);

    % 距离条件验证
    if dist_to_center >= min_center_dist
        start = candidate;
    end
    attempt = attempt + 1;
end

if isempty(start)
    error('无法找到符合条件的起点，请检查地图障碍物分布');
end

%% 终点生成
goal = [];
attempt = 0;
while isempty(goal) && attempt < max_attempts
    % 生成随机候选点
    candidate = [randi(height), randi(width)];

    % 障碍物检测
    if map(candidate(1), candidate(2)) ~= 0
        attempt = attempt + 1;
        continue;
    end

    % 计算到起点距离
    dist_to_start = norm(candidate - start);

    % 距离条件验证
    if dist_to_start >= min_start_goal_dist
        goal = candidate;
    end
    attempt = attempt + 1;
end

if isempty(goal)
    error('无法找到符合条件的终点，请检查地图障碍物分布');
end

%% 可视化验证（调试用）
% figure;
% imshow(~map); % 反色显示地图
% hold on;
% plot(start(2), start(1), 'go', 'MarkerSize', 10, 'LineWidth', 2);
% plot(goal(2), goal(1), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
% title('起终点生成验证');
end