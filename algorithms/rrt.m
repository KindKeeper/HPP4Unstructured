function [refined_path, rrt_tree] = rrt(map, global_path, start, goal)

% RRT参数设置
max_iter = 5000;      % 最大迭代次数
step_size = 2;         % 生长步长
goal_bias = 0.12;      % 目标偏向概率
search_radius = 3;     % 局部搜索半径

% 初始化树结构（修复start识别问题）
tree.vertices = start;  % 使用传入的start参数
tree.edges = [];
tree.costs = 0;

% 沿着全局路径设置中间目标点
intermediate_goals = global_path(2:end-1, :);  % 排除起点和终点

% 主循环
for iter = 1:max_iter
    % 以一定概率选择最终目标点
    if rand < goal_bias
        target_point = goal;
    else
        % 在全局路径附近随机采样
        rand_idx = randi(size(intermediate_goals,1));
        target_point = intermediate_goals(rand_idx,:) + randn(1,2)*search_radius;
        target_point = clamp_point(target_point, size(map));
    end

    % 寻找最近节点
    [nearest_node, nearest_id] = find_nearest_node(tree.vertices, target_point);

    % 向目标方向生长
    new_point = steer(nearest_node, target_point, step_size);
    new_point = clamp_point(new_point, size(map));

    % 碰撞检测
    if check_collision(map, nearest_node, new_point)
        continue;
    end

    % 添加新节点
    tree.vertices = [tree.vertices; new_point];
    tree.edges = [tree.edges; nearest_id, size(tree.vertices,1)];
    tree.costs = [tree.costs; tree.costs(nearest_id) + norm(new_point - nearest_node)];

    % 检查是否到达目标
    if norm(new_point - goal) < step_size
        % 添加最终目标点
        tree.vertices = [tree.vertices; goal];
        tree.edges = [tree.edges; size(tree.vertices,1)-1, size(tree.vertices,1)];
        break;
    end
end

% 提取最终路径
refined_path = extract_path(tree, start, goal);

% 路径后处理
refined_path = smooth_path(map, refined_path);

rrt_tree = tree;
end

% 新增辅助函数
function clamped = clamp_point(point, map_size)
clamped = point;
clamped(1) = max(1, min(map_size(1)-1, round(clamped(1))));
clamped(2) = max(1, min(map_size(2)-1, round(clamped(2))));
end

% 辅助函数 --------------------------------------------------
function [nearest, idx] = find_nearest_node(nodes, point)
distances = vecnorm(nodes - point, 2, 2);
[~, idx] = min(distances);
nearest = nodes(idx,:);
end

function new_point = steer(from, to, max_dist)
direction = to - from;
distance = norm(direction);
if distance <= max_dist
    new_point = to;
else
    new_point = from + (direction/distance)*max_dist;
end
end

function collision = check_collision(map, p1, p2)
collision = false;
points = interpolate_points(p1, p2);
for i = 1:size(points,1)
    x = round(points(i,1));
    y = round(points(i,2));
    if map(x,y) == 1
        collision = true;
        return;
    end
end
end

function points = interpolate_points(p1, p2)
num_points = ceil(norm(p2 - p1)*2);
x = linspace(p1(1), p2(1), num_points);
y = linspace(p1(2), p2(2), num_points);
points = unique([x' y'], 'rows', 'stable');
end

function path = extract_path(tree, start, goal)
path = goal;
current_node = size(tree.vertices,1);

while current_node ~= 1
    parent = tree.edges(tree.edges(:,2) == current_node, 1);
    path = [tree.vertices(parent,:); path];
    current_node = parent;
end
path = [start; path];
end

function smoothed_path = smooth_path(map, path)
smoothed_path = path(1,:);
current_index = 1;

while current_index < size(path,1)
    next_index = size(path,1);
    while next_index > current_index + 1
        if ~check_collision(map, path(current_index,:), path(next_index,:))
            break;
        end
        next_index = next_index - 1;
    end
    smoothed_path = [smoothed_path; path(next_index,:)];
    current_index = next_index;
end
end
