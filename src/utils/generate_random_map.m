function map = generate_random_map(varargin)
% 生成随机栅格地图（可指定参数）
% 输入参数可选：
%   1. [height, width]
%   2. height, width
%   3. height, width, obstacle_density
%   4. height, width, obstacle_density, seed

% 参数解析
switch nargin
    case 0
        height = 50; width = 50; density = 0.25; seed = 'shuffle';
    case 1
        height = varargin{1}(1); width = varargin{1}(2);
        density = 0.25; seed = 'shuffle';
    case 2
        height = varargin{1}; width = varargin{2};
        density = 0.25; seed = 'shuffle';
    case 3
        [height, width, density] = varargin{1:3};
        seed = 'shuffle';
    case 4
        [height, width, density, seed] = varargin{1:4};
end

% 初始化随机数生成器
rng(seed);

% 生成基础障碍物
base_map = rand(height, width) < density;

% 形态学处理优化障碍物分布
se = strel('square', 3);
processed_map = imdilate(base_map, se); % 膨胀操作形成连续障碍

% 转换为标准地图格式（0-自由空间，1-障碍物）
map = double(processed_map);

% 连通性保障检测
if ~check_map_connectivity(map)
    warning('检测到不连通地图，重新生成中...');
    map = generate_random_map(height, width, density*0.9); % 递归降低密度
end
end

function is_connected = check_map_connectivity(map)
% 使用洪水填充算法检查基础连通性
[height, width] = size(map);
start = [ceil(height/2), ceil(width/2)];

% 寻找最近的自由点作为检测起点
if map(start(1), start(2)) ~= 0
    [x, y] = find(map == 0);
    if isempty(x)
        is_connected = false;
        return;
    end
    start = [x(1), y(1)];
end

% 执行洪水填充
filled = imfill(~map, start);
is_connected = all(filled(:) | map(:)); % 检查是否充满整个自由空间
end