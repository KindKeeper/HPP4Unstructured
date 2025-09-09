function length = calculate_path_length(path)
length = 0;
for i = 2:size(path, 1)
    segment = sqrt((path(i,1) - path(i-1,1))^2 + (path(i,2) - path(i-1,2))^2);
    length = length + segment;
end
end
