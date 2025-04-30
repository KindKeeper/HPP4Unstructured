function astar_main()
    clear;
    clc;
    
map = [ 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 1 1 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 1 0 0 0 1 1 1 1 0 0 0 0 0 0 0 0 1 1 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0;
        0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0;
        0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 1 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0;
        0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 1 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 1 1 1 0 0 0 0 1 0 0 0 0;
        0 0 0 0 0 0 0 0 1 1 1 1 1 1 1 1 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0;
        0 0 0 0 1 0 0 0 0 0 0 0 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 1 0 0 0 1 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 1 0 1 0 0 0 0;
        0 0 0 0 0 1 1 1 1 0 0 0 0 0 0 0 0 0 1 0 0 0 0 1 1 1 0 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 1 0 0 0 0 0;
        0 0 0 0 0 1 1 1 1 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 1 0 0 0 1 1 1 1 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 1 1 1 1 0 0 0 0 1 1 1 1 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0;
        0 0 0 0 0 1 1 1 1 0 0 0 0 0 1 1 0 0 1 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 1 1 1 1 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 1 0 0 0 0 1 0 0 0 1 1 0 0 1 0 0 0 0 0 1 1 0 0 0;
        0 0 0 0 0 1 1 1 1 0 0 1 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 1 0 0 0;
        0 0 0 0 0 1 1 1 1 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 1 0 0 1 0 0 0 0 1 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 1 1 0 0 1 1 1 1 1 1 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 1 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 1 1 0 0 0 0 0 0 0;
        1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 0 0;
        1 1 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0;
        0 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0 0 1 0 0 0 1 0 1 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 1 0 0 0 0 0 1 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 1 0 0 0;
        0 0 0 0 0 0 1 0 0 0 1 0 0 0 0 0 0 0 0 1 0 0 0 1 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 1 0 0 1 1 0 0;
        0 0 1 0 0 0 0 0 0 0 0 0 1 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 1 0 0 1 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 1 1 0 0 0 0;
        0 0 0 0 1 1 0 0 1 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 1 1 0 0 0 0 0 0 0 1 0 0 0 0;
        0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 0 1 1 0 0 0 1 1 1 0 0 0 0 1 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 0 0 1 1 1 1 0 0 1 1 1 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 1 0 0 0 0 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 1 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 1 1 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 1 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 0 0 0 0 0 0 0 0;
        0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 1 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0;
        0 1 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 0;
        0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 1 1 1 1 0 0 0 0 0 0 0 0 1 1 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 1 1 0 0 0;
        0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 1 1 1 1 1 1 0 0 0 0 0 0 0 1 1 1 1 0 0 0 0 0 0 1 1 1 0 0 0 0 0 0 0 0 0;];
    
    start = [4, 4];
    goal = [47, 47];
    
    % 初始路径规划
    [path1, explored1] = astar(map, start, goal);
    
    % 添加新障碍物（设置为2以区分）
    new_obstacles = [20, 13; 20, 12; 21, 12; 33, 27; 32, 27];
    for i = 1:size(new_obstacles, 1)
        map(new_obstacles(i,1), new_obstacles(i,2)) = 2; % 设置为2
    end
    
      % 改进的路径阻断检测（提前3格检测）
    new_start = start;
    blocked = false;
    lookahead = 3; % 设置提前检测的格子数
    
    for i = 1:size(path1,1)
        % 检测未来3格是否包含障碍物
        end_index = min(i+lookahead, size(path1,1)); % 防止数组越界
        future_path = path1(i:end_index, :);
        
        % 检查未来路径段是否包含障碍物
        [is_blocked, block_idx] = check_blockage(future_path, new_obstacles);
        
        if is_blocked
            blocked = true;
            % 使用当前点作为新起点（提前响应）
            new_start = path1(max(1,i-1), :); % 确保索引有效
            blockage_index = i-1; % 记录检测到障碍物的索引点
            disp(['在位置 ', num2str(new_start), ' 检测到前方障碍物，开始重规划']);
            break;
        end
    end
    
    % 重新规划路径
    if blocked
    [replanned_path, explored2] = astar(map, new_start, goal);
        
        % 创建完整的重规划路径：从起点到检测点的原始路径 + 从检测点到目标的重规划路径
        if ~isempty(replanned_path)
            path2 = [path1(1:blockage_index, :); replanned_path(2:end, :)]; % 避免重复点
        else
            path2 = path1(1:blockage_index, :); % 如果重规划失败，至少保留到检测点的路径
        end
    else
        path2 = [];
        explored2 = [];
    end
    
    % 可视化（显示新增障碍物和两次路径）
    visualize_combined(map, path1, path2,start, goal);

    visualize_obstacle_distances(map, path1, path2);

    % 可视化拐点和转弯角度 (NEW)
    visualize_turning_points(path1, path2);
    
    % 计算并显示路径统计信息 (NEW)
    display_path_statistics(path1, path2);
end

% 新函数：显示路径统计信息
function display_path_statistics(path1, path2)
    % 计算路径1的统计数据
    [turns_path1, angles_path1, ~] = calculate_turning_points(path1);
    total_angle_path1 = sum(abs(angles_path1)) * (180/pi);
    avg_angle_path1 = mean(abs(angles_path1)) * (180/pi);
    path_length1 = calculate_path_length(path1);
    
    % 计算路径2的统计数据
    if ~isempty(path2)
        [turns_path2, angles_path2, ~] = calculate_turning_points(path2);
        total_angle_path2 = sum(abs(angles_path2)) * (180/pi);
        avg_angle_path2 = mean(abs(angles_path2)) * (180/pi);
        path_length2 = calculate_path_length(path2);
    else
        turns_path2 = 0;
        total_angle_path2 = 0;
        avg_angle_path2 = 0;
        path_length2 = 0;
    end
    
    % 在命令窗口显示详细统计信息
    fprintf('\n--------- 路径统计分析 ---------\n');
    fprintf('初始路径:\n');
    fprintf('  - 总长度: %.2f\n', path_length1);
    fprintf('  - 拐点数量: %d\n', length(angles_path1));
    fprintf('  - 总转弯角度: %.2f°\n', total_angle_path1);
    fprintf('  - 平均转弯角度: %.2f°\n', avg_angle_path1);
    
    if ~isempty(path2)
        fprintf('\n重规划路径:\n');
        fprintf('  - 总长度: %.2f\n', path_length2);
        fprintf('  - 拐点数量: %d\n', length(angles_path2));
        fprintf('  - 总转弯角度: %.2f°\n', total_angle_path2);
        fprintf('  - 平均转弯角度: %.2f°\n', avg_angle_path2);
        
        % 计算差异百分比
        length_diff = ((path_length2 - path_length1) / path_length1) * 100;
        turns_diff = ((length(angles_path2) - length(angles_path1)) / length(angles_path1)) * 100;
        angle_diff = ((total_angle_path2 - total_angle_path1) / total_angle_path1) * 100;
        
        fprintf('\n路径比较 (重规划 vs 初始):\n');

        % Fix for the length difference reporting
        if length_diff > 0
            length_change_direction = '增加';
        else
            length_change_direction = '减少';
        end
        fprintf('  - 长度变化: %.2f%% (%s)\n', abs(length_diff), length_change_direction);

        % Fix for the turns count difference reporting
        if turns_diff > 0
         turns_change_direction = '增加';
        else
         turns_change_direction = '减少';
        end
        fprintf('  - 拐点数量变化: %.2f%% (%s)\n', abs(turns_diff), turns_change_direction);

        % Fix for the total angle difference reporting
        if angle_diff > 0
            angle_change_direction = '增加';
        else
            angle_change_direction = '减少';
        end
        fprintf('  - 总转弯角度变化: %.2f%% (%s)\n', abs(angle_diff), angle_change_direction);
            end
            fprintf('---------------------------------\n');
        end

% 新函数：计算路径长度
function length = calculate_path_length(path)
    length = 0;
    for i = 2:size(path, 1)
        segment = sqrt((path(i,1) - path(i-1,1))^2 + (path(i,2) - path(i-1,2))^2);
        length = length + segment;
    end
end

% [Rest of the original functions remain the same]

% 添加辅助函数：计算路径的拐点和角度
function [turn_count, angles, turn_indices] = calculate_turning_points(path)
    turn_count = 0;
    angles = [];
    turn_indices = [];
    
    % Need at least 3 points to have a turning point
    if size(path, 1) < 3
        return;
    end
    
    % Compute movement vectors between consecutive points
    for i = 2:size(path, 1)-1
        % Vector from previous point to current point
        v1 = [path(i,1) - path(i-1,1), path(i,2) - path(i-1,2)];
        % Vector from current point to next point
        v2 = [path(i+1,1) - path(i,1), path(i+1,2) - path(i,2)];
        
        % Normalize vectors
        v1_norm = v1 / norm(v1);
        v2_norm = v2 / norm(v2);
        
        % Calculate dot product to find cosine of angle
        cos_angle = dot(v1_norm, v2_norm);
        
        % Clamp to valid range for acos
        cos_angle = max(-1, min(1, cos_angle));
        
        % Calculate angle
        angle = acos(cos_angle);
        
        % Calculate cross product to determine sign of angle
        cross_z = v1_norm(1)*v2_norm(2) - v1_norm(2)*v2_norm(1);
        if cross_z < 0
            angle = -angle;
        end
        
        % If angle is not close to 0, it's a turning point
        if abs(angle) > 0.01  % Small threshold to account for floating-point errors
            turn_count = turn_count + 1;
            angles = [angles; angle];
            turn_indices = [turn_indices; i];
        end
    end
end

% 新增辅助函数：检查路径段是否被阻断
function [is_blocked, block_idx] = check_blockage(path_segment, obstacles)
    is_blocked = false;
    block_idx = 0;
    for k = 1:size(path_segment,1)
        if ismember(path_segment(k,:), obstacles, 'rows')
            is_blocked = true;
            block_idx = k;
            return;
        end
    end
end

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

function visualize_combined(map, path1, path2, start, goal)
    figure;
    hold on;
    
    % 创建颜色映射 - 调整新增障碍物的颜色为更鲜明的橙红色
    cmap = [1 1 1;           % 0: 白色（可行区域）
            0 0 0;           % 1: 黑色（原始障碍物）
            1 0.3 0];        % 2: 橙红色（新增障碍物）- 调整为更明亮的颜色
    colormap(cmap);
    
    % 绘制地图
    [rows, cols] = size(map);
    imagesc(1:cols, 1:rows, map);
    
    % 绘制栅格线
    line_color = [0.4 0.4 0.4];
    for y = 0.5:rows+0.5
        line([0.5, cols+0.5], [y, y], 'Color', line_color, 'LineWidth', 0.5);
    end
    for x = 0.5:cols+0.5
        line([x, x], [0.5, rows+0.5], 'Color', line_color, 'LineWidth', 0.5);
    end
    
    % 创建图例句柄数组
    legend_handles = [];
    legend_items = {};
    
    % 绘制路径1
    if ~isempty(path1)
        h3 = plot(path1(:,2), path1(:,1), 'Color', [0 1 0], 'LineWidth', 2.5);
        legend_handles = [legend_handles, h3];
        legend_items = [legend_items, {'初始路径'}];
    end
    
    % 绘制路径2 - 使更小的间距
    if ~isempty(path2)
        h4 = plot(path2(:,2), path2(:,1), 'Color', [1 0 0], 'LineWidth', 2.5, 'LineStyle', ':');
        legend_handles = [legend_handles, h4];
        legend_items = [legend_items, {'重规划路径'}];
    end
    
    % 标记起点、终点（使用五角星）
    plot(start(2), start(1), 'p', 'MarkerSize', 12, 'MarkerFaceColor', [1 0 0], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    plot(goal(2), goal(1), 'p', 'MarkerSize', 12, 'MarkerFaceColor', [1 1 0], 'MarkerEdgeColor', 'k', 'LineWidth', 1.5);
    
    % 为起点和终点创建图例句柄
    h5 = plot(NaN, NaN, 'p', 'MarkerSize', 10, 'MarkerFaceColor', [1 0 0], 'MarkerEdgeColor', 'k');
    h6 = plot(NaN, NaN, 'p', 'MarkerSize', 10, 'MarkerFaceColor', [1 1 0], 'MarkerEdgeColor', 'k');
    legend_handles = [legend_handles, h5, h6];
    legend_items = [legend_items, {'起点', '终点'}];

    % 添加新增障碍物的图例 - 使用与colormap中相同的颜色
    new_obstacle_color = [1 0.3 0]; % 橙红色，与colormap中值为2的颜色一致
    h_new_obstacle = plot(NaN, NaN, 's', 'MarkerSize', 10, 'MarkerFaceColor', [1 0.3 0], 'MarkerEdgeColor', 'none');
    legend_handles = [legend_handles, h_new_obstacle];
    legend_items = [legend_items, {'新增障碍物'}];
    
    % 设置轴属性
    axis equal tight;
    set(gca, 'YDir', 'reverse', 'Visible', 'off', 'XLim', [0.5 cols+0.5], 'YLim', [0.5 rows+0.5]);
    
    % 添加图例 - 设置为横向，每行4个，指定位置，字体和字号
    lgd = legend(legend_handles, legend_items, 'Orientation', 'horizontal', 'NumColumns', 4);
    
    % 设置图例位置
    set(lgd, 'Position', [0.223220973782772, 0.047998211173221, 0.573782771535581, 0.052142857142857]);
    
    % 设置图例字体为宋体，字号10
    set(lgd, 'FontName', '宋体', 'FontSize', 12);
    
    % 添加标题
    title('动态路径规划可视化结果', 'FontName', '宋体', 'FontSize', 14, 'FontWeight', 'bold');
    
    hold off;
end

% 添加到现有代码中的函数

function visualize_obstacle_distances(map, path1, path2)
    % 创建一个新图形窗口
    figure('Name', '路径与障碍物距离统计', 'Position', [100, 100, 800, 500]);
    
    % 初始化结果数组
    distances_path1 = zeros(size(path1, 1), 1);
    distances_path2 = zeros(size(path2, 1), 1);
    
    % 计算每个路径点到最近障碍物的距离
    for i = 1:size(path1, 1)
        distances_path1(i) = calculate_min_obstacle_distance(map, path1(i,:));
    end
    
    for i = 1:size(path2, 1)
        distances_path2(i) = calculate_min_obstacle_distance(map, path2(i,:));
    end
    
    % 创建子图1: 展示沿路径的距离变化
    subplot(2, 1, 1);
    
    % 绘制初始路径的距离
    plot(1:length(distances_path1), distances_path1, 'o-', 'Color', [0, 0.7, 0], 'LineWidth', 2, 'MarkerSize', 4, 'MarkerFaceColor', [0, 0.7, 0]);
    hold on;
    
    % 绘制重规划路径的距离
    if ~isempty(path2)
        plot(1:length(distances_path2), distances_path2, 's-', 'Color', [0.8, 0, 0], 'LineWidth', 2, 'MarkerSize', 4, 'MarkerFaceColor', [0.8, 0, 0]);
    end
    
    % 添加网格线和图例
    grid on;
    legend({'初始路径', '重规划路径'}, 'Location', 'best', 'FontName', '宋体', 'FontSize', 12);
    
    % 设置标题和标签
    title('路径上各点到最近障碍物的距离', 'FontName', '宋体', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('路径节点索引', 'FontName', '宋体', 'FontSize', 12);
    ylabel('到最近障碍物的距离', 'FontName', '宋体', 'FontSize', 12);
    
    % 创建子图2: 使用路径位置作为x轴（距离起点的实际距离）
    subplot(2, 1, 2);
    
    % 计算沿路径的累积距离
    cum_dist_path1 = calculate_cumulative_distance(path1);
    cum_dist_path2 = calculate_cumulative_distance(path2);
    
    % 绘制初始路径的距离
    plot(cum_dist_path1, distances_path1, 'o-', 'Color', [0, 0.7, 0], 'LineWidth', 2, 'MarkerSize', 4, 'MarkerFaceColor', [0, 0.7, 0]);
    hold on;
    
    % 绘制重规划路径的距离
    if ~isempty(path2)
        plot(cum_dist_path2, distances_path2, 's-', 'Color', [0.8, 0, 0], 'LineWidth', 2, 'MarkerSize', 4, 'MarkerFaceColor', [0.8, 0, 0]);
    end
    
    % 添加网格线和图例
    grid on;
    legend({'初始路径', '重规划路径'}, 'Location', 'best', 'FontName', '宋体', 'FontSize', 12);
    
    % 设置标题和标签
    title('沿路径距离的障碍物距离变化', 'FontName', '宋体', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('沿路径的累积距离', 'FontName', '宋体', 'FontSize', 12);
    ylabel('到最近障碍物的距离', 'FontName', '宋体', 'FontSize', 12);
    
    % 添加一些基本的统计信息
    stats_text = sprintf(['初始路径: 最小距离 = %.2f, 平均距离 = %.2f\n', ...
                         '重规划路径: 最小距离 = %.2f, 平均距离 = %.2f'], ...
                         min(distances_path1), mean(distances_path1), ...
                         min(distances_path2), mean(distances_path2));
    
    annotation('textbox', [0.15, 0.01, 0.7, 0.05], 'String', stats_text, ...
               'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
               'FontName', '宋体', 'FontSize', 10);
end

% 计算一个点到地图中最近障碍物的距离
function min_dist = calculate_min_obstacle_distance(map, point)
    [rows, cols] = size(map);
    min_dist = inf;
    
    % 遍历地图中的所有障碍物点
    for i = 1:rows
        for j = 1:cols
            if map(i, j) >= 1  % 障碍物（包括原始障碍物和新增障碍物）
                dist = sqrt((point(1) - i)^2 + (point(2) - j)^2);
                min_dist = min(min_dist, dist);
            end
        end
    end
end

% 计算路径的累积距离
function cum_dist = calculate_cumulative_distance(path)
    cum_dist = zeros(size(path, 1), 1);
    
    for i = 2:size(path, 1)
        % 计算当前点与前一个点的欧几里得距离
        step_dist = sqrt(sum((path(i,:) - path(i-1,:)).^2));
        cum_dist(i) = cum_dist(i-1) + step_dist;
    end
end

function visualize_turning_points(path1, path2)
    % Create a new figure window
    figure('Name', '路径拐点与转弯角度分析', 'Position', [100, 100, 800, 600]);
    
    % Calculate turning points and angles for both paths
    [turns_path1, angles_path1, turn_indices_path1] = calculate_turning_points(path1);
    [turns_path2, angles_path2, turn_indices_path2] = calculate_turning_points(path2);
    
    % Calculate total turning angle (sum of absolute angles)
    total_angle_path1 = sum(abs(angles_path1));
    total_angle_path2 = sum(abs(angles_path2));
    
    % Subplot 1: Turning angles along the path
    subplot(2, 1, 1);
    
    % Plot turning angles for initial path
    if ~isempty(angles_path1)
        plot(turn_indices_path1, angles_path1 * (180/pi), 'o-', 'Color', [0, 0.7, 0], 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', [0, 0.7, 0]);
        hold on;
    end
    
    % Plot turning angles for replanned path
    if ~isempty(path2) && ~isempty(angles_path2)
        plot(turn_indices_path2, angles_path2 * (180/pi), 's-', 'Color', [0.8, 0, 0], 'LineWidth', 2, 'MarkerSize', 6, 'MarkerFaceColor', [0.8, 0, 0]);
    end
    
    % Add grid and legend
    grid on;
    legend({'初始路径', '重规划路径'}, 'Location', 'best', 'FontName', '宋体', 'FontSize', 12);
    
    % Set title and labels
    title('路径转弯角度分布', 'FontName', '宋体', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('路径节点索引', 'FontName', '宋体', 'FontSize', 12);
    ylabel('转弯角度 (度)', 'FontName', '宋体', 'FontSize', 12);
    
    % Add horizontal line at 0 degrees
    yline(0, '--', 'Color', [0.5, 0.5, 0.5]);
    
    % Subplot 2: Path visualization with turning points highlighted
    subplot(2, 1, 2);
    
    % Plot initial path
    plot(path1(:,2), path1(:,1), '-', 'Color', [0, 0.7, 0], 'LineWidth', 2);
    hold on;
    
    % Highlight turning points on initial path
    if ~isempty(turn_indices_path1)
        scatter(path1(turn_indices_path1,2), path1(turn_indices_path1,1), 60, 'o', 'filled', 'MarkerFaceColor', [0, 0.7, 0], 'MarkerEdgeColor', 'k');
    end
    
    % Plot replanned path if available
    if ~isempty(path2)
        plot(path2(:,2), path2(:,1), ':', 'Color', [0.8, 0, 0], 'LineWidth', 2);
        
        % Highlight turning points on replanned path
        if ~isempty(turn_indices_path2)
            scatter(path2(turn_indices_path2,2), path2(turn_indices_path2,1), 60, 's', 'filled', 'MarkerFaceColor', [0.8, 0, 0], 'MarkerEdgeColor', 'k');
        end
    end
    
    % Add grid and legend
    grid on;
    legend({'初始路径', '初始路径拐点', '重规划路径', '重规划路径拐点'}, ...
           'Location', 'best', 'FontName', '宋体', 'FontSize', 10);
    
    % Set title and labels
    title('路径拐点可视化', 'FontName', '宋体', 'FontSize', 14, 'FontWeight', 'bold');
    xlabel('X坐标', 'FontName', '宋体', 'FontSize', 12);
    ylabel('Y坐标', 'FontName', '宋体', 'FontSize', 12);
    axis equal;
    
    % Add statistical information as annotation
    stats_text = sprintf(['初始路径: 拐点数量 = %d, 总转弯角度 = %.2f°\n', ...
                         '重规划路径: 拐点数量 = %d, 总转弯角度 = %.2f°'], ...
                         length(angles_path1), total_angle_path1 * (180/pi), ...
                         length(angles_path2), total_angle_path2 * (180/pi));
    
    annotation('textbox', [0.15, 0.01, 0.7, 0.05], 'String', stats_text, ...
               'EdgeColor', 'none', 'HorizontalAlignment', 'center', ...
               'FontName', '宋体', 'FontSize', 10);
end