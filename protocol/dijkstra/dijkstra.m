function [path, explored] = dijkstra(map, start, goal)
[rows, cols] = size(map);
directions = [-1 0; 1 0; 0 -1; 0 1; -1 -1; -1 1; 1 -1; 1 1];
is_diagonal = [false; false; false; false; true; true; true; true];

openList = [];
closedList = zeros(rows, cols);
explored = [];

g_score = inf(rows, cols);
parent = zeros(rows, cols, 2);

if start(1) == goal(1) && start(2) == goal(2)
    path = start;
    return;
end

if map(start(1), start(2)) ~= 0 || map(goal(1), goal(2)) ~= 0
    path = [];
    disp('起点或终点是障碍物，无法找到路径！');
    return;
end

% 初始化只设置g_score
g_score(start(1), start(2)) = 0;
openList = [openList; start g_score(start(1), start(2))]; % 存储g_score

while ~isempty(openList)
    % 按g_score排序开放列表
    [~, idx] = sort(openList(:, 3)); % 第三列现在是g_score
    openList = openList(idx, :);
    current = openList(1, 1:2);

    if current(1) == goal(1) && current(2) == goal(2)
        path = reconstruct_path(parent, current);
        return;
    end

    openList(1, :) = [];
    closedList(current(1), current(2)) = 1;
    explored = [explored; current];

    for i = 1:size(directions, 1)
        neighbor = current + directions(i, :);

        if neighbor(1) < 1 || neighbor(1) > rows || ...
                neighbor(2) < 1 || neighbor(2) > cols
            continue;
        end

        if map(neighbor(1), neighbor(2)) ~= 0 || closedList(neighbor(1), neighbor(2)) == 1
            continue;
        end

        if is_diagonal(i)
            dx = directions(i, 1);
            dy = directions(i, 2);
            if map(current(1) + dx, current(2)) == 1 && ...
                    map(current(1), current(2) + dy) == 1
                continue;
            end
        end

        % 仅计算移动代价（去掉启发式）
        if is_diagonal(i)
            cost = sqrt(2);
        else
            cost = 1;
        end
        tentative_g = g_score(current(1), current(2)) + cost;

        if tentative_g < g_score(neighbor(1), neighbor(2))
            parent(neighbor(1), neighbor(2), :) = current;
            g_score(neighbor(1), neighbor(2)) = tentative_g;

            in_open_list = false;
            for j = 1:size(openList, 1)
                if openList(j, 1) == neighbor(1) && openList(j, 2) == neighbor(2)
                    openList(j, 3) = g_score(neighbor(1), neighbor(2)); % 更新g_score
                    in_open_list = true;
                    break;
                end
            end

            if ~in_open_list
                openList = [openList; neighbor g_score(neighbor(1), neighbor(2))];
            end
        end
    end

    if isempty(openList)
        path = [];
        disp('无法找到路径！所有可达节点已被探索。');
        return;
    end
end

path = [];
disp('无法找到路径！');
end

% 重建路径
function path = reconstruct_path(parent, current)
path = current;
while true
    p = reshape(parent(current(1), current(2), :), 1, 2);
    % 如果已经到达起点 (父节点为[0,0])
    if p(1) == 0 && p(2) == 0
        break;
    end
    path = [p; path];
    current = p;
end
end
