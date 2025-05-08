function [blocked, new_start, blockage_index] = detect_blockage(path, obstacles, map, strategy)
% 输入参数新增map用于检测地图障碍
blocked = false;
new_start = path(1,:);
blockage_index = 1;

% 策略控制检测范围
if strcmpi(strategy, 'dynamic')
    lookahead = round(size(path,1)*0.1);
else
    lookahead = 3;
end

for i = 1:size(path,1)
    end_index = min(i+lookahead, size(path,1));
    future_path = path(i:end_index, :);
    
    % 修改检测逻辑：同时检查地图和动态障碍
    [is_blocked, hit_pos] = check_blockage(future_path, obstacles, map);
    
    if is_blocked
        blocked = true;
        blockage_index = max(1, i-1);
        new_start = path(blockage_index, :);
        disp(['障碍物预警：在路径点', num2str(new_start),...
             ' 检测到障碍物坐标', num2str(hit_pos)]);
        break;
    end
end
end