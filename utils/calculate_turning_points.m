function [turn_count, angles, turn_indices] = calculate_turning_points(path)
turn_count = 0;
angles = [];
turn_indices = [];

% Need at least 3 points to have a turning point
if size(path, 1) < 3
    return;
end

% Compute movement vectors between consecutive points
for i = 2:size(path, 1)-1
    % Vector from previous point to current point
    v1 = [path(i,1) - path(i-1,1), path(i,2) - path(i-1,2)];
    % Vector from current point to next point
    v2 = [path(i+1,1) - path(i,1), path(i+1,2) - path(i,2)];

    % Normalize vectors
    v1_norm = v1 / norm(v1);
    v2_norm = v2 / norm(v2);

    % Calculate dot product to find cosine of angle
    cos_angle = dot(v1_norm, v2_norm);

    % Clamp to valid range for acos
    cos_angle = max(-1, min(1, cos_angle));

    % Calculate angle
    angle = acos(cos_angle);

    % Calculate cross product to determine sign of angle
    cross_z = v1_norm(1)*v2_norm(2) - v1_norm(2)*v2_norm(1);
    if cross_z < 0
        angle = -angle;
    end

    % If angle is not close to 0, it's a turning point
    if abs(angle) > 0.01  % Small threshold to account for floating-point errors
        turn_count = turn_count + 1;
        angles = [angles; angle];
        turn_indices = [turn_indices; i];
    end
end
end
