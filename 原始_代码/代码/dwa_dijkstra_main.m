function dijkstra_main()
    clear;
    clc;
    
    % 创建初始地图
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
    % Visualize both global and refined paths
    [global_path1, explored1] = dijkstra(map, start, goal);
   
    if isempty(global_path1)
        disp('Global path planning failed!');
        return;
    end
    
    [refined_path1] = apply_dwa(map, global_path1, goal);

    % 添加新障碍物（设置为2以区分）
    new_obstacles = [20, 13; 20, 12; 21, 12; 33, 27; 32, 27];
    for i = 1:size(new_obstacles, 1)
        map(new_obstacles(i,1), new_obstacles(i,2)) = 2; % 设置为2
    end

      % 改进的路径阻断检测（提前3格检测）
    new_start = start;
    blocked = false;
    lookahead = 3; % 设置提前检测的格子数
    
     for i = 1:size(refined_path1,1)
        % 检测未来3格是否包含障碍物
        end_index = min(i+lookahead, size(refined_path1,1)); % 防止数组越界
        future_path = refined_path1(i:end_index, :);
        
        % 检查未来路径段是否包含障碍物
        [is_blocked, block_idx] = check_blockage(future_path, new_obstacles);
        
        if is_blocked
            blocked = true;
            % 使用当前点作为新起点（提前响应）
            new_start = refined_path1(max(1,i-1), :); % 确保索引有效
            blockage_index = i-1; % 记录检测到障碍物的索引点
            disp(['在位置 ', num2str(new_start), ' 检测到前方障碍物，开始重规划']);
            break;
        end
    end
    
    % 重新规划路径
    if blocked
        [global_path2, explored2] = dijkstra(map, new_start, goal);
   
     if isempty(global_path2)
        disp('Global path planning failed!');
        return;
     end
    
    [replanned_path] = apply_dwa(map, global_path2, goal);

    % 创建完整的重规划路径：从起点到检测点的原始路径 + 从检测点到目标的重规划路径
        if ~isempty(replanned_path)
            refined_path2 = [refined_path1(1:blockage_index, :); replanned_path(2:end, :)]; % 避免重复点
        else
            refined_path2 = refined_path1(1:blockage_index, :); % 如果重规划失败，至少保留到检测点的路径
        end
    else
        refined_path2 = [];
        explored2 = [];
    end
    
    % 可视化（显示新增障碍物和两次路径）
    visualize_combined(map, refined_path1, refined_path2, start, goal);

    visualize_obstacle_distances(map, refined_path1, refined_path2);

    % 可视化拐点和转弯角度 (NEW)
    visualize_turning_points(refined_path1, refined_path2);
    
    % 计算并显示路径统计信息 (NEW)
    display_path_statistics(refined_path1, refined_path2);
end


% 新函数：显示路径统计信息
function display_path_statistics(refined_path1, refined_path2)
    % 计算路径1的统计数据
    [turns_path1, angles_path1, ~] = calculate_turning_points(refined_path1);
    total_angle_path1 = sum(abs(angles_path1)) * (180/pi);
    avg_angle_path1 = mean(abs(angles_path1)) * (180/pi);
    path_length1 = calculate_path_length(refined_path1);
    
    % 计算路径2的统计数据
    if ~isempty(refined_path2)
        [turns_path2, angles_path2, ~] = calculate_turning_points(refined_path2);
        total_angle_path2 = sum(abs(angles_path2)) * (180/pi);
        avg_angle_path2 = mean(abs(angles_path2)) * (180/pi);
        path_length2 = calculate_path_length(refined_path2);
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
    
    if ~isempty(refined_path2)
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
function length = calculate_path_length(refined_path)
    length = 0;
    for i = 2:size(refined_path, 1)
        segment = sqrt((refined_path(i,1) - refined_path(i-1,1))^2 + (refined_path(i,2) - refined_path(i-1,2))^2);
        length = length + segment;
    end
end

% [Rest of the original functions remain the same]

% 添加辅助函数：计算路径的拐点和角度
function [turn_count, angles, turn_indices] = calculate_turning_points(refined_path)
    turn_count = 0;
    angles = [];
    turn_indices = [];
    
    % Need at least 3 points to have a turning point
    if size(refined_path, 1) < 3
        return;
    end
    
    % Compute movement vectors between consecutive points
    for i = 2:size(refined_path, 1)-1
        % Vector from previous point to current point
        v1 = [refined_path(i,1) - refined_path(i-1,1), refined_path(i,2) - refined_path(i-1,2)];
        % Vector from current point to next point
        v2 = [refined_path(i+1,1) - refined_path(i,1), refined_path(i+1,2) - refined_path(i,2)];
        
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

function [path, explored] = dijkstra(map, start, goal)
    [rows, cols] = size(map);
    directions = [-1 0; 1 0; 0 -1; 0 1; -1 -1; -1 1; 1 -1; 1 1];
    is_diagonal = [false; false; false; false; true; true; true; true];

    openList = [];
    closedList = zeros(rows, cols);
    explored = [];
    
    % 只保留g_score矩阵
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

function visualize_combined(map, refined_path1, refined_path2, start, goal)
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
    if ~isempty(refined_path1)
        h3 = plot(refined_path1(:,2), refined_path1(:,1), 'Color', [0 1 0], 'LineWidth', 2.5);
        legend_handles = [legend_handles, h3];
        legend_items = [legend_items, {'初始路径'}];
    end
    
    % 绘制路径2 - 使更小的间距
    if ~isempty(refined_path2)
        h4 = plot(refined_path2(:,2), refined_path2(:,1), 'Color', [1 0 0], 'LineWidth', 2.5, 'LineStyle', ':');
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
    new_obstacle_color = [0.5 0.4 0.5]; % 橙红色，与colormap中值为2的颜色一致
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

function visualize_obstacle_distances(map, refined_path1, refined_path2)
    % 创建一个新图形窗口
    figure('Name', '路径与障碍物距离统计', 'Position', [100, 100, 800, 500]);
    
    % 初始化结果数组
    distances_path1 = zeros(size(refined_path1, 1), 1);
    distances_path2 = zeros(size(refined_path2, 1), 1);
    
    % 计算每个路径点到最近障碍物的距离
    for i = 1:size(refined_path1, 1)
        distances_path1(i) = calculate_min_obstacle_distance(map, refined_path1(i,:));
    end
    
    for i = 1:size(refined_path2, 1)
        distances_path2(i) = calculate_min_obstacle_distance(map, refined_path2(i,:));
    end
    
    % 创建子图1: 展示沿路径的距离变化
    subplot(2, 1, 1);
    
    % 绘制初始路径的距离
    plot(1:length(distances_path1), distances_path1, 'o-', 'Color', [0, 0.7, 0], 'LineWidth', 2, 'MarkerSize', 4, 'MarkerFaceColor', [0, 0.7, 0]);
    hold on;
    
    % 绘制重规划路径的距离
    if ~isempty(refined_path2)
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
    cum_dist_path1 = calculate_cumulative_distance(refined_path1);
    cum_dist_path2 = calculate_cumulative_distance(refined_path2);
    
    % 绘制初始路径的距离
    plot(cum_dist_path1, distances_path1, 'o-', 'Color', [0, 0.7, 0], 'LineWidth', 2, 'MarkerSize', 4, 'MarkerFaceColor', [0, 0.7, 0]);
    hold on;
    
    % 绘制重规划路径的距离
    if ~isempty(refined_path2)
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
function cum_dist = calculate_cumulative_distance(refined_path)
    cum_dist = zeros(size(refined_path, 1), 1);
    
    for i = 2:size(refined_path, 1)
        % 计算当前点与前一个点的欧几里得距离
        step_dist = sqrt(sum((refined_path(i,:) - refined_path(i-1,:)).^2));
        cum_dist(i) = cum_dist(i-1) + step_dist;
    end
end

function visualize_turning_points(refined_path1, refiend_path2)
    % Create a new figure window
    figure('Name', '路径拐点与转弯角度分析', 'Position', [100, 100, 800, 600]);
    
    % Calculate turning points and angles for both paths
    [turns_path1, angles_path1, turn_indices_path1] = calculate_turning_points(refined_path1);
    [turns_path2, angles_path2, turn_indices_path2] = calculate_turning_points(refiend_path2);
    
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
    if ~isempty(refiend_path2) && ~isempty(angles_path2)
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
    plot(refined_path1(:,2), refined_path1(:,1), '-', 'Color', [0, 0.7, 0], 'LineWidth', 2);
    hold on;
    
    % Highlight turning points on initial path
    if ~isempty(turn_indices_path1)
        scatter(refined_path1(turn_indices_path1,2), refined_path1(turn_indices_path1,1), 60, 'o', 'filled', 'MarkerFaceColor', [0, 0.7, 0], 'MarkerEdgeColor', 'k');
    end
    
    % Plot replanned path if available
    if ~isempty(refiend_path2)
        plot(refiend_path2(:,2), refiend_path2(:,1), ':', 'Color', [0.8, 0, 0], 'LineWidth', 2);
        
        % Highlight turning points on replanned path
        if ~isempty(turn_indices_path2)
            scatter(refiend_path2(turn_indices_path2,2), refiend_path2(turn_indices_path2,1), 60, 's', 'filled', 'MarkerFaceColor', [0.8, 0, 0], 'MarkerEdgeColor', 'k');
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