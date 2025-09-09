function execute_visualization(map, path1, path2, start, goal)
% 统一可视化函数
visualize_combined(map, path1, path2, start, goal);
visualize_obstacle_distances(map, path1, path2);
visualize_turning_points(path1, path2);
display_path_statistics(path1, path2);
end