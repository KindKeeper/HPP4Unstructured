function [path, explored] = astar(map, start, goal)
[rows, cols] = size(map);
directions = [-1 0; 1 0; 0 -1; 0 1; -1 -1; -1 1; 1 -1; 1 1];
is_diagonal = [false; false; false; false; true; true; true; true];

openList = [];
closedList = zeros(rows, cols);
explored = [];

g_score = inf(rows, cols);
f_score = inf(rows, cols);
parent = zeros(rows, cols, 2);

if isequal(start, goal)
    path = start;
    return;
end

if map(start(1), start(2)) == 1 || map(goal(1), goal(2)) == 1
    path = [];
    return;
end

g_score(start(1), start(2)) = 0;
f_score(start(1), start(2)) = heuristic(start, goal);
openList = [openList; start f_score(start(1), start(2))];

while ~isempty(openList)
    [~, idx] = sort(openList(:,3));
    openList = openList(idx, :);
    current = openList(1, 1:2);

    if isequal(current, goal)
        path = reconstruct_path(parent, current);
        return;
    end

    openList(1, :) = [];
    closedList(current(1), current(2)) = 1;
    explored = [explored; current];

    for i = 1:size(directions,1)
        neighbor = current + directions(i,:);

        if neighbor(1) < 1 || neighbor(1) > rows || neighbor(2) < 1 || neighbor(2) > cols
            continue;
        end

        if map(neighbor(1), neighbor(2)) >= 1 || closedList(neighbor(1), neighbor(2)) == 1
            continue;
        end

        if is_diagonal(i)
            dx = directions(i,1);
            dy = directions(i,2);
            adj1 = map(current(1)+dx, current(2)) >= 1;
            adj2 = map(current(1), current(2)+dy) >= 1;
            if adj1 && adj2
                continue;
            end
        end

        if is_diagonal(i)
            tentative_g = g_score(current(1), current(2)) + sqrt(2);
        else
            tentative_g = g_score(current(1), current(2)) + 1;
        end

        if tentative_g < g_score(neighbor(1), neighbor(2))
            parent(neighbor(1), neighbor(2), :) = current;
            g_score(neighbor(1), neighbor(2)) = tentative_g;
            f_score(neighbor(1), neighbor(2)) = tentative_g + heuristic(neighbor, goal);

            in_open = false;
            for j = 1:size(openList,1)
                if openList(j,1) == neighbor(1) && openList(j,2) == neighbor(2)
                    openList(j,3) = f_score(neighbor(1), neighbor(2));
                    in_open = true;
                    break;
                end
            end
            if ~in_open
                openList = [openList; neighbor f_score(neighbor(1), neighbor(2))];
            end
        end
    end
end
path = [];
end

function h = heuristic(node, goal)
h = sqrt((node(1)-goal(1))^2 + (node(2)-goal(2))^2);
end

function path = reconstruct_path(parent, current)
path = current;
while true
    p = reshape(parent(current(1), current(2), :), 1, 2);
    if all(p == [0,0])
        break;
    end
    path = [p; path];
    current = p;
end
end
