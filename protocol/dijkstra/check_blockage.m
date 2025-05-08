function [is_blocked, block_idx] = check_blockage(path_segment, obstacles)
is_blocked = false;
block_idx = 0;
for k = 1:size(path_segment,1)
    if ismember(path_segment(k,:), obstacles, 'rows')
        is_blocked = true;
        block_idx = k;
        return;
    end
end
end
