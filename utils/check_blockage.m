function [is_blocked, hit_pos] = check_blockage(future_path, obstacles, map)
% 输入参数新增map，输出命中位置
is_blocked = false;
hit_pos = [0, 0];

% 优先检查地图障碍
for i = 1:size(future_path,1)
    % 转换到矩阵坐标系
    map_coord = fliplr(future_path(i,:));
    
    % 检查矩阵边界
    if map_coord(1)<1 || map_coord(1)>size(map,1) ||...
       map_coord(2)<1 || map_coord(2)>size(map,2)
        is_blocked = true;
        hit_pos = future_path(i,:);
        return;
    end
    
    % 检查地图障碍（包含动态障碍）
    if map(map_coord(1), map_coord(2)) ~= 0
        is_blocked = true;
        hit_pos = future_path(i,:);
        return;
    end
end

% 补充检查动态障碍列表（冗余检测）
for j = 1:size(obstacles,1)
    if ismember(obstacles(j,:), future_path, 'rows')
        is_blocked = true;
        hit_pos = obstacles(j,:);
        return;
    end
end
end