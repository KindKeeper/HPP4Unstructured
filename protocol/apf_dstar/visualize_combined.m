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
