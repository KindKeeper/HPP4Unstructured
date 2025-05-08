% 生成地图并可视化
clc;
map = generateLayeredMap([300 300],4);
show(map)

% 生成三层地图并叠加
function finalMap = generateLayeredMap(mapSize,resolution)

% ===== 生成层1: 迷宫基础地图 =====
passageWidth = 4;    % 通道宽度（网格数）
wallThickness = 1;   % 墙壁厚度（网格数）
layer1 = mapMaze(passageWidth, wallThickness, ...
    'MapSize', mapSize, ...
    'MapResolution', resolution);

% ===== 生成叠加层 =====
% 层2: 增加障碍物
layer2 = mapClutter(64, {'Box', 'Circle'},...
    'MapSize', mapSize,...
    'MapResolution', resolution);

% 层3: 减少障碍物
layer3 = mapClutter(32, {'Box', 'Circle'},...
    'MapSize', mapSize,...
    'MapResolution', resolution);

% 新层4: 增加障碍物（可设置不同参数）
layer4 = mapClutter(16, {'Box', 'Circle'},... % 新增45个障碍
    'MapSize', mapSize,...
    'MapResolution', resolution);

% 新层5: 减少障碍物（可设置不同参数）
layer5 = mapClutter(8, {'Box', 'Circle'},... % 新增25个移除区域
    'MapSize', mapSize,...
    'MapResolution', resolution);

% ===== 矩阵数据获取 =====
% 转换所有层为逻辑矩阵
mat1 = logical(occupancyMatrix(layer1));
mat2 = logical(occupancyMatrix(layer2));
mat3 = logical(occupancyMatrix(layer3));
mat4 = logical(occupancyMatrix(layer4));
mat5 = logical(occupancyMatrix(layer5));

% ===== 多层叠加逻辑 =====
combinedMat = mat1 | mat2 | mat4;   % 叠加所有增加层
combinedMat = combinedMat & ~mat3;  % 移除第三层区域
combinedMat = combinedMat & ~mat5;  % 新增：移除第五层区域

% ===== 生成最终地图 =====
finalMap = binaryOccupancyMap(combinedMat, resolution);
finalMap.GridLocationInWorld = [0 0];
end