function [path, explored] = dstar(map, start, goal)
[rows, cols] = size(map);

NEW = 0;
OPEN = 1;
CLOSED = 2;

directions = [-1 0; 1 0; 0 -1; 0 1; -1 -1; -1 1; 1 -1; 1 1];
costs = [1; 1; 1; 1; sqrt(2); sqrt(2); sqrt(2); sqrt(2)];

state = zeros(rows, cols);
t = inf(rows, cols);
h = zeros(rows, cols);
b = zeros(rows, cols, 2);

open_list = [];

max_explored = rows * cols;
explored = zeros(max_explored, 2);
explored_count = 0;

% 最大迭代次数防止无限循环
MAX_ITERATIONS = rows * cols * 10;
iteration_count = 0;

% 计算启发式值
for i = 1:rows
    for j = 1:cols
        h(i,j) = sqrt((i-goal(1))^2 + (j-goal(2))^2);
    end
end

% 特殊情况判断
if isequal(start, goal)
    path = start;
    return;
end

if map(start(1), start(2)) >= 1 || map(goal(1), goal(2)) >= 1
    path = [];
    disp('起点或终点在障碍物上，无法找到路径！');
    return;
end

% 初始化目标节点
t(goal(1), goal(2)) = 0;
state(goal(1), goal(2)) = OPEN;

% 将目标节点加入开放列表，键值为0
open_list = [goal(1), goal(2), 0];

% 主循环
while ~isempty(open_list) && state(start(1), start(2)) ~= CLOSED && iteration_count < MAX_ITERATIONS
    iteration_count = iteration_count + 1;

    % 获取键值最小的节点
    [~, min_idx] = min(open_list(:,3));
    x = open_list(min_idx, 1);
    y = open_list(min_idx, 2);
    open_list(min_idx, :) = [];

    % 记录探索节点
    explored_count = explored_count + 1;
    if explored_count <= max_explored
        explored(explored_count, :) = [x, y];
    end

    % 如果节点已关闭则跳过
    if state(x, y) == CLOSED
        continue;
    end

    % 将当前节点标记为已关闭
    state(x, y) = CLOSED;

    % 检查所有邻居节点
    for i = 1:length(directions)
        nx = x + directions(i, 1);
        ny = y + directions(i, 2);

        % 检查边界
        if nx < 1 || nx > rows || ny < 1 || ny > cols
            continue;
        end

        % 检查障碍物
        if map(nx, ny) >= 1
            continue;
        end

        % 对角线移动需要检查拐角
        if costs(i) > 1
            if map(x, ny) >= 1 || map(nx, y) >= 1
                continue;
            end
        end

        cost = costs(i);

        if t(nx, ny) > t(x, y) + cost
            t(nx, ny) = t(x, y) + cost;
            b(nx, ny, :) = [x, y];

            new_key = t(nx, ny) + h(nx, ny);

            node_idx = find(open_list(:,1) == nx & open_list(:,2) == ny);

            if isempty(node_idx)
                open_list = [open_list; [nx, ny, new_key]];
                state(nx, ny) = OPEN;
            else
                open_list(node_idx, 3) = new_key;
            end
        end
    end
end

% 截取有效的探索节点
explored = explored(1:explored_count, :);

% 检查是否达到最大迭代次数
if iteration_count >= MAX_ITERATIONS
    disp(['警告：达到最大迭代次数 ', num2str(MAX_ITERATIONS), '，算法提前终止']);
end

% 重建路径
if state(start(1), start(2)) == CLOSED
    path = reconstruct_path(start, goal, b);
else
    path = [];
    disp('未找到路径！');
end
end

% 根据后继节点重建路径
function path = reconstruct_path(start, goal, b)
current = start;
path = start;

% 设置最大路径长度以防止无限循环
max_path_length = numel(b) / 2;
path_length = 1;

% 从起点到目标
while ~isequal(current, goal) && path_length < max_path_length
    % 获取后继节点
    next_row = b(current(1), current(2), 1);
    next_col = b(current(1), current(2), 2);

    % 如果没有有效的后继节点，退出
    if next_row == 0 && next_col == 0
        disp('路径重建失败，无法找到完整路径！');
        return;
    end

    next_node = [next_row, next_col];

    % 检测循环路径
    if any(ismember(path, next_node, 'rows'))
        disp('检测到路径循环，路径重建中断！');
        return;
    end

    % 添加到路径
    path = [path; next_node];
    current = next_node;
    path_length = path_length + 1;
end

% 检查是否达到最大路径长度
if path_length >= max_path_length
    disp('警告：达到最大路径长度，路径可能不完整');
end
end
