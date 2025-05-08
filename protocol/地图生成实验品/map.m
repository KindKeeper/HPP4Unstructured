%% 主程序
function main()
clc;
rng(42);
mapSize = 512;

terrain = generateTerrain(mapSize);

startPoint = [50, 50];
endPoint = [450, 450];
[roadMask, smoothPath] = generateRoad(terrain, startPoint, endPoint);

obstacles = poissonDiscSampling(mapSize, 8, 30);

finalMap = combineLayers(terrain, roadMask, obstacles);
visualizeMap(finalMap);
end

%% 地形生成函数（必须位于主函数之后）
function terrain = generateTerrain(size)
[X,Y] = meshgrid(1:size, 1:size);
noise = perlinNoise2D(X, Y, 100, 6, 0.5, 2);

terrain = zeros(size);
terrain(noise < -0.5) = 3;   % 水域
terrain(noise >= -0.5 & noise < 0) = 2; % 沼泽
terrain(noise >= 0) = 1;     % 可通行区域
end

%% 改进的Perlin噪声生成（使用梯度噪声）
%% 修正后的Perlin噪声生成函数
function noise = perlinNoise2D(X, Y, scale, octaves, persistence, lacunarity)
    noise = zeros(size(X));
    maxAmp = 0;
    amp = 1;
    
    for i = 1:octaves
        freq = scale / (lacunarity^(i-1));
        gridSize = ceil(scale/freq) + 1;
        
        % 生成随机梯度场
        angles = 2*pi * rand(gridSize);
        gradX = cos(angles);
        gradY = sin(angles);
        
        % 计算网格坐标
        xg = floor(X/freq);
        yg = floor(Y/freq);
        dx = X/freq - xg;
        dy = Y/freq - yg;
        
        % 边界处理
        xg = mod(xg-1, gridSize) + 1;
        yg = mod(yg-1, gridSize) + 1;
        
        % 计算四个角点贡献
        n00 = dotGradient(gradX, gradY, xg, yg, dx, dy, gridSize);
        n01 = dotGradient(gradX, gradY, xg, yg+1, dx, dy-1, gridSize);
        n10 = dotGradient(gradX, gradY, xg+1, yg, dx-1, dy, gridSize);
        n11 = dotGradient(gradX, gradY, xg+1, yg+1, dx-1, dy-1, gridSize);
        
        % 双线性插值
        u = fade(dx);
        v = fade(dy);
        nx0 = lerp(n00, n10, u);
        nx1 = lerp(n01, n11, u);
        noise = noise + amp * lerp(nx0, nx1, v);
        
        maxAmp = maxAmp + amp;
        amp = amp * persistence;
    end
    noise = noise / maxAmp;
end

%% 辅助函数
function val = dotGradient(gradX, gradY, xg, yg, dx, dy, gridSize)
    xg = mod(xg-1, gridSize) + 1;
    yg = mod(yg-1, gridSize) + 1;
    g = gradX(xg, yg).*dx + gradY(xg, yg).*dy;
    val = g .* (6*dx.^5 - 15*dx.^4 + 10*dx.^3); % 改进型缓和曲线
end

function t = fade(t)
    t = 6*t.^5 - 15*t.^4 + 10*t.^3; % Perlin改进的缓和曲线
end

function y = lerp(a, b, x)
    y = a + x.*(b - a);
end

%% 优化的A*算法实现
function path = aStar(costMap, start, goal)
[h, w] = size(costMap);
openSet = PriorityQueue();
cameFrom = containers.Map('KeyType','char','ValueType','any');
gScore = inf(h,w);

startKey = mat2str(start);
openSet.insert(start, 0);
gScore(start(2), start(1)) = 0;

while ~openSet.isempty()
    [current, ~] = openSet.pop();
    if isequal(current, goal)
        path = reconstructPath(cameFrom, current);
        return;
    end

    neighbors = getNeighbors(current, costMap);
    for i = 1:size(neighbors,1)
        neighbor = neighbors(i,:);
        tentative_gScore = gScore(current(2), current(1)) + ...
            costMap(neighbor(2), neighbor(1));

        if tentative_gScore < gScore(neighbor(2), neighbor(1))
            cameFrom(mat2str(neighbor)) = current;
            gScore(neighbor(2), neighbor(1)) = tentative_gScore;
            fScore = tentative_gScore + sum(abs(neighbor - goal));
            if ~openSet.contains(neighbor)
                openSet.insert(neighbor, fScore);
            end
        end
    end
end
error('路径不存在');
end

%% 泊松圆盘采样优化实现
function points = poissonDiscSampling(mapSize, radius, k)
cellSize = floor(radius/sqrt(2));
gridSize = ceil(mapSize/cellSize) + 1;
grid = cell(gridSize, gridSize);

start = [randi(mapSize), randi(mapSize)];
points = start;
grid{getGridIdx(start, cellSize)} = start;
active = 1;

while ~isempty(active)
    idx = randi(length(active));
    point = points(active(idx),:);
    found = false;

    for i = 1:k
        angle = 2*pi*rand;
        r = radius + radius*rand;
        newPoint = round(point + r*[cos(angle), sin(angle)]);

        if all(newPoint > 0) && all(newPoint <= mapSize)
            gridKey = getGridIdx(newPoint, cellSize);
            if checkNeighbors(newPoint, grid, gridKey, radius, cellSize)
                points = [points; newPoint];
                grid{gridKey(1), gridKey(2)} = [grid{gridKey(1), gridKey(2)}; newPoint];
                active = [active; size(points,1)];
                found = true;
                break;
            end
        end
    end

    if ~found
        active(idx) = [];
    end
end
end

%% 道路绘制优化（使用Bresenham算法）
function roadMask = drawRoad(path, width, mapSize)
roadMask = zeros(mapSize);
for i = 1:size(path,1)-1
    line = bresenham(path(i,:), path(i+1,:));
    roadMask(sub2ind(size(roadMask), line(:,2), line(:,1))) = 1;
end
roadMask = imdilate(roadMask, strel('disk', width));
end

%% 辅助函数和类定义


function idx = getGridIdx(point, cellSize)
idx = [floor(point(1)/cellSize)+1, floor(point(2)/cellSize)+1];
end

function valid = checkNeighbors(point, grid, gridKey, radius, cellSize)
for x = max(1,gridKey(1)-2):min(size(grid,1),gridKey(1)+2)
    for y = max(1,gridKey(2)-2):min(size(grid,2),gridKey(2)+2)
        if ~isempty(grid{x,y})
            dists = sum((grid{x,y} - point).^2, 2);
            if any(dists < radius^2)
                valid = false;
                return
            end
        end
    end
end
valid = true;
end