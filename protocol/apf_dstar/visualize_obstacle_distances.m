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
