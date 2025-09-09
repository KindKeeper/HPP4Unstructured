function [map, start, goal] = generate_valid_environment(config)
% 复用generate_start_goal实现环境生成
max_attempts = 10;
for attempt = 1:max_attempts
    map = occupancyMatrix(generate_layered_map(config.map_size, config.map_resolution));

    try
        [start, goal] = generate_start_goal(map);
        start = fliplr(start); % 坐标转换
        goal = fliplr(goal);
        return;
    catch
        if attempt == max_attempts
            error('无法生成有效环境，请调整参数');
        end
    end
end
end
