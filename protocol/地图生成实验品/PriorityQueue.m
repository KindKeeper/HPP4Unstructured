classdef PriorityQueue < handle
    properties
        elements = [];
        priorities = [];
    end
    
    methods
        function insert(obj, element, priority)
            obj.elements = [obj.elements; element];
            obj.priorities = [obj.priorities; priority];
        end
        
        function [element, priority] = pop(obj)
            [~, idx] = min(obj.priorities);
            element = obj.elements(idx,:);
            priority = obj.priorities(idx);
            obj.elements(idx,:) = [];
            obj.priorities(idx) = [];
        end
        
        function bool = contains(obj, element)
            bool = any(all(obj.elements == element,2));
        end
        
        function bool = isempty(obj)
            bool = isempty(obj.elements);
        end
    end
end