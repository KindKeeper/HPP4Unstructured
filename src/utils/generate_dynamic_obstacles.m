function obstacles = generate_dynamic_obstacles(map, start, goal, num)
% 复用check_blockage的逻辑实现障碍生成
obstacles = zeros(num, 2);
valid_pos = find(map == 0);
[row, col] = ind2sub(size(map), valid_pos);

% 排除起终点坐标
start_idx = sub2ind(size(map), start(2), start(1));
goal_idx = sub2ind(size(map), goal(2), goal(1));
valid_pos = setdiff(valid_pos, [start_idx, goal_idx]);

for i = 1:num
    if isempty(valid_pos)
        break;
    end
    sel = randi(length(valid_pos));
    [r, c] = ind2sub(size(map), valid_pos(sel));
    obstacles(i,:) = [r, c];
    valid_pos(sel) = [];
end
obstacles = obstacles(any(obstacles,2),:); % 移除未生成的障碍
end