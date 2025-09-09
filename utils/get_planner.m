function planner = get_planner(name, type)
% 获取规划器函数句柄
% 输入参数：
%   name - 算法名称字符串
%   type - 规划器类型（'global'/'local'）
% 输出参数：
%   planner - 对应的函数句柄

global_planners = containers.Map(...
    {'astar', 'dijkstra', 'dstar', 'rrt'}, ...
    {@astar, @dijkstra, @dstar, @rrt});

local_planners = containers.Map(...
    {'dwa', 'apf'}, ...
    {@dwa, @apf});

switch lower(type)
    case 'global'
        if isKey(global_planners, lower(name))
            planner = global_planners(lower(name));
        else
            error('不支持的全局规划器: %s', name);
        end
    case 'local'
        if isKey(local_planners, lower(name))
            planner = local_planners(lower(name));
        else
            error('不支持的局部规划器: %s', name);
        end
    otherwise
        error('未知规划器类型: %s', type);
end
end
