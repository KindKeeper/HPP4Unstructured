function refined_path = apf(map, global_path, goal)
% Parameters for APF
k_att = 5.0;       % Attractive force gain
k_rep = 15.0;     % Repulsive force gain
rho_0 = 6.0;       % Influence range of obstacles
step_size = 0.1;   % Step size for path adjustment
max_iterations = 100; % Maximum iterations for APF adjustment

% Initialize refined path with global path
refined_path = global_path;
[rows, cols] = size(map);

% Process each point in the global path except start and goal
for i = 2:size(global_path, 1)-1
    current_point = global_path(i, :);

    for iter = 1:max_iterations
        % Calculate attractive force toward the goal
        goal_vector = goal - current_point;
        dist_to_goal = norm(goal_vector);

        % Normalize the goal vector
        if dist_to_goal > 0
            goal_vector = goal_vector / dist_to_goal;
        end

        % Attractive force
        F_att = k_att * goal_vector;

        % Initialize repulsive force
        F_rep = [0, 0];

        % Calculate repulsive forces from obstacles
        for r = 1:rows
            for c = 1:cols
                if map(r, c) == 1  % If it's an obstacle
                    obstacle_pos = [r, c];
                    obstacle_vector = current_point - obstacle_pos;
                    dist_to_obstacle = norm(obstacle_vector);

                    % Apply repulsive force within influence range
                    if dist_to_obstacle <= rho_0 && dist_to_obstacle > 0
                        % Normalize the obstacle vector
                        obstacle_vector = obstacle_vector / dist_to_obstacle;

                        % Calculate repulsive force
                        F_rep_mag = k_rep * (1/dist_to_obstacle - 1/rho_0) * (1/(dist_to_obstacle^2));
                        F_rep = F_rep + F_rep_mag * obstacle_vector;
                    end
                end
            end
        end

        % Total force
        F_total = F_att + F_rep;

        % Normalize the force if it's too large
        force_magnitude = norm(F_total);
        if force_magnitude > step_size
            F_total = F_total * (step_size / force_magnitude);
        end

        % Update the current point
        new_point = current_point + F_total;

        % Check if the new point is valid (not out of bounds or in an obstacle)
        new_r = round(new_point(1));
        new_c = round(new_point(2));

        if new_r >= 1 && new_r <= rows && new_c >= 1 && new_c <= cols && map(new_r, new_c) == 0
            current_point = new_point;
        else
            % If invalid, stop iterations for this point
            break;
        end

        % Check if we've converged
        if norm(F_total) < 0.1
            break;
        end
    end

    % Update the refined path with the adjusted point
    refined_path(i, :) = round(current_point);
end

% Post-process to remove duplicate points and ensure smoothness
refined_path = remove_duplicates_and_smooth(refined_path);
end

% Remove duplicate points and smooth the path
function smoothed_path = remove_duplicates_and_smooth(path)
% Remove duplicate consecutive points
i = 1;
smoothed_path = path(1,:);

while i < size(path, 1)
    i = i + 1;
    if ~isequal(path(i,:), smoothed_path(end,:))
        smoothed_path = [smoothed_path; path(i,:)];
    end
end
end
