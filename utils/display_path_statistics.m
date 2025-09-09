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
