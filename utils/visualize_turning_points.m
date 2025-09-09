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