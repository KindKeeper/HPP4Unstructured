function stitched_path = stitch_paths(original, new, stitch_index)
% 路径拼接函数
if ~isempty(new)
    stitched_path = [original(1:stitch_index, :); new(2:end, :)];
else
    stitched_path = original(1:stitch_index, :);
end
end
