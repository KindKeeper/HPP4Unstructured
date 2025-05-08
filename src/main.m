function main()
clear; clc;

%% 算法配置模块
config.global_planner = 'astar';      % 可选: astar/dijkstra/dstar/rrt
config.local_planner = 'dwa';         % 可选: dwa/apf
config.replan_strategy = 'dynamic';   % 重规划策略: dynamic/static
config.map_size = [20, 20];           % 新增：地图尺寸 [行,列]
config.map_resolution = 4;           % 新增：地图分辨率（每米多少格）
config.obstacle_density = 0.01;        % 新增：障碍物密度
config.new_obstacles_num = 5;         % 新增：动态障碍物数量

%% 环境初始化配置
try
    addpath(genpath('planner'));
    addpath(genpath('utils'));
catch
    error('路径初始化失败，请检查目录结构完整性');
end

%% 地图与任务初始化
% === 修改点1：生成随机地图 ===
[map, start, goal] = generate_valid_environment(config);

%% 第一阶段路径规划
global_planner = get_planner(config.global_planner, 'global');
[global_path1, explored1] = global_planner(map, start, goal);

if isempty(global_path1)
    disp('全局路径规划失败，请检查地图连通性！');
    return;
end

local_planner = get_planner(config.local_planner, 'local');
[refined_path1] = local_planner(map, global_path1, goal);

%% 动态环境变化模拟
% === 修改点2：生成随机动态障碍 ===
new_obstacles = generate_dynamic_obstacles(map, start, goal, config.new_obstacles_num);
for i = 1:size(new_obstacles, 1)
    map(new_obstacles(i,1), new_obstacles(i,2)) = 2; % 保持原有标记方式
end

%% 路径阻断预检测机制
[blocked, new_start, blockage_index] = ...
    detect_blockage(refined_path1, new_obstacles, map, config.replan_strategy); % 新增map参数

%% 第二阶段路径重规划
if blocked
    [global_path2, explored2] = global_planner(map, new_start, goal);

    if isempty(global_path2)
        disp('路径重规划失败，导航任务终止！');
        return;
    end

    [replanned_path] = local_planner(map, global_path2, goal);
    refined_path2 = path_stitching(refined_path1, replanned_path, blockage_index);
else
    refined_path2 = [];
end

%% 可视化与输出模块
execute_visualization(map, global_path1, refined_path2, start, goal);
end
