% navigation_demo.m
% This file shows how the fused RTS positions could be used for
% basic robot navigation on the highway route.
%
% It does three things:
%   1. Places some fake obstacles on the route (to simulate road works)
%   2. Checks if the robot path goes too close to any obstacle
%   3. If yes, it shifts the path sideways to avoid it (simple detour)
%
% Note: the obstacles are not real - I made them up to demonstrate
% the idea. The path planning is also simple (just a sideways offset),
% not a proper algorithm like A* or RRT. This is just a demo to show
% how the fused position output could feed into a navigation system.


function navigation_demo(data, xs, cfg)

    % get the fused position and velocity from the RTS smoother output
    pos_x = xs(:,1);   % X position
    pos_y = xs(:,3);   % Y position
    vel_x = xs(:,2);   % X velocity
    vel_y = xs(:,4);   % Y velocity

    % calculate heading angle from velocity direction
    % atan2 gives the angle in radians from the X axis
    heading = atan2(vel_y, vel_x);

    N = length(pos_x);

    % -------------------------------------------------------------------------
    % Step 1 - place some fake obstacles on the route
    % -------------------------------------------------------------------------
    % Each row is [x_centre, y_centre, radius_in_metres]
    % I picked positions roughly along the route at steps 320, 680, 950
    % and added an offset so they sit near but not exactly on the path

    obstacles = [
        data.true_x(320) + 45,  data.true_y(320) + 30,  55;
        data.true_x(680) - 55,  data.true_y(680) - 45,  48;
        data.true_x(950) + 40,  data.true_y(950) + 38,  60;
    ];

    fprintf('  %d obstacles placed (these are fake - for demo only).\n', ...
            size(obstacles,1));

    % -------------------------------------------------------------------------
    % Step 2 - check for collisions and plan detours
    % -------------------------------------------------------------------------
    % Start with the fused path as the planned path
    planned_x = pos_x;
    planned_y = pos_y;

    safety_margin = 15;   % extra buffer on top of obstacle radius (metres)
    detour_offset = 80;   % how far sideways to shift during a detour (metres)
    any_detour    = false;

    for i = 1:size(obstacles,1)

        obs_x = obstacles(i,1);
        obs_y = obstacles(i,2);
        obs_r = obstacles(i,3) + safety_margin;

        % find the closest point on the path to this obstacle
        dist_to_obs = sqrt((pos_x - obs_x).^2 + (pos_y - obs_y).^2);
        [~, closest_idx] = min(dist_to_obs);

        if dist_to_obs(closest_idx) < obs_r
            fprintf('  Obstacle %d is too close at step %d - creating detour.\n', ...
                    i, closest_idx);
            any_detour = true;

            % define a window of steps around the obstacle to detour
            win_start = max(1,  closest_idx - 30);
            win_end   = min(N,  closest_idx + 40);

            % calculate a perpendicular direction to the path
            % this is just the direction 90 degrees to the travel direction
            dx = pos_x(win_end) - pos_x(win_start);
            dy = pos_y(win_end) - pos_y(win_start);
            path_len = norm([dx; dy]);

            if path_len < 1e-6
                continue;  % skip if the path segment is too short
            end

            % unit perpendicular vector (rotate 90 degrees)
            perp = [-dy; dx] / path_len;

            % shift the planned path sideways in the detour window
            planned_x(win_start:win_end) = planned_x(win_start:win_end) + detour_offset * perp(1);
            planned_y(win_start:win_end) = planned_y(win_start:win_end) + detour_offset * perp(2);
        end
    end

    if ~any_detour
        fprintf('  No obstacles close enough to the path - no detour needed.\n');
    end

    % -------------------------------------------------------------------------
    % Step 3 - plot the route with obstacles and planned path
    % -------------------------------------------------------------------------
    figure('Name', 'Figure 8 - Navigation Demo', 'NumberTitle', 'off', ...
           'Position', [100 80 1400 680]);

    sgtitle({'Navigation Demo: Fused RTS Path + Obstacle Avoidance', ...
             '(Obstacles are synthetic - for illustration only)'}, ...
            'FontSize', 13, 'FontWeight', 'bold');

    cT = cfg.colors.truth;
    cR = cfg.colors.rts;
    cP = [0.93 0.69 0.13];   % yellow/orange for planned path

    % left subplot - full route
    subplot(1,2,1);
    hold on; grid on; axis equal;

    plot(data.true_x, data.true_y, '-',  'Color', cT, 'LineWidth', 2.5, ...
         'DisplayName', 'Ground Truth');
    plot(pos_x, pos_y, '--', 'Color', cR, 'LineWidth', 1.8, ...
         'DisplayName', 'RTS Fused Path');
    plot(planned_x, planned_y, '-', 'Color', cP, 'LineWidth', 2.5, ...
         'DisplayName', 'Planned Path (with detours)');

    % draw obstacle circles
    for i = 1:size(obstacles,1)
        viscircles([obstacles(i,1), obstacles(i,2)], obstacles(i,3), ...
                   'Color', 'r', 'LineWidth', 1.5);
        text(obstacles(i,1), obstacles(i,2)+obstacles(i,3)+20, ...
             sprintf('Obs %d', i), 'Color', 'r', 'FontSize', 9, ...
             'HorizontalAlignment', 'center');
    end

    % mark start and goal
    plot(pos_x(1),   pos_y(1),   'gs', 'MarkerSize', 12, 'MarkerFaceColor', 'g', ...
         'DisplayName', 'Start');
    plot(pos_x(end), pos_y(end), 'r^', 'MarkerSize', 12, 'MarkerFaceColor', 'r', ...
         'DisplayName', 'Goal');

    xlabel('X [m]', 'FontSize', 11);
    ylabel('Y [m]', 'FontSize', 11);
    title('Full 43 km Route', 'FontSize', 11);
    legend('Location', 'best', 'FontSize', 9);

    % right subplot - zoomed view on first 500 steps
    subplot(1,2,2);
    hold on; grid on; axis equal;

    sl = 1:min(500, N);
    plot(data.true_x(sl), data.true_y(sl), '-',  'Color', cT, 'LineWidth', 2.5, ...
         'DisplayName', 'Ground Truth');
    plot(pos_x(sl), pos_y(sl), '--', 'Color', cR, 'LineWidth', 1.8, ...
         'DisplayName', 'RTS Fused Path');
    plot(planned_x(sl), planned_y(sl), '-', 'Color', cP, 'LineWidth', 2.5, ...
         'DisplayName', 'Planned Path');

    % only draw obstacles that are near this zoomed window
    for i = 1:size(obstacles,1)
        near = sqrt((pos_x(sl) - obstacles(i,1)).^2 + ...
                    (pos_y(sl) - obstacles(i,2)).^2);
        if min(near) < obstacles(i,3) + 200
            viscircles([obstacles(i,1), obstacles(i,2)], obstacles(i,3), ...
                       'Color', 'r', 'LineWidth', 1.5);
        end
    end

    xlabel('X [m]', 'FontSize', 11);
    ylabel('Y [m]', 'FontSize', 11);
    title('Zoomed View (First 500 Steps)', 'FontSize', 11);
    legend('Location', 'best', 'FontSize', 9);

    fprintf('  -> Figure 8: Navigation demo plot done.\n');

    % -------------------------------------------------------------------------
    % Step 4 - simple live animation of the robot following the planned path
    % -------------------------------------------------------------------------
    fprintf('  Starting robot animation (close window to skip)...\n');

    anim_fig = figure('Name', 'Robot Animation - Fused Navigation', ...
                      'NumberTitle', 'off', 'Position', [150 100 1200 700]);
    hold on; grid on; axis equal;

    plot(planned_x, planned_y, '-', 'Color', cP, 'LineWidth', 2.0, ...
         'DisplayName', 'Planned Path');
    plot(pos_x, pos_y, '--', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.0, ...
         'DisplayName', 'Fused RTS Path');

    for i = 1:size(obstacles,1)
        viscircles([obstacles(i,1), obstacles(i,2)], obstacles(i,3), ...
                   'Color', 'r', 'LineStyle', '--');
    end

    xlabel('X [m]', 'FontSize', 11);
    ylabel('Y [m]', 'FontSize', 11);
    legend('Location', 'northwest', 'FontSize', 9);

    robot_marker = [];
    robot_arrow  = [];
    robot_size   = 40;    % visual size of robot triangle in metres
    step_skip    = 10;    % only draw every 10th step for speed

    for k = 1:step_skip:N

        % check figure still exists (user may have closed it)
        if ~isvalid(anim_fig)
            fprintf('  Animation window closed by user.\n');
            break;
        end

        % remove previous robot drawing
        if ~isempty(robot_marker) && isvalid(robot_marker)
            delete(robot_marker);
        end
        if ~isempty(robot_arrow) && isvalid(robot_arrow)
            delete(robot_arrow);
        end

        x  = planned_x(k);
        y  = planned_y(k);
        th = heading(k);

        % draw robot as a small triangle pointing in the heading direction
        front  = [x + robot_size*cos(th),        y + robot_size*sin(th)];
        left   = [x + robot_size*cos(th + 2.4),  y + robot_size*sin(th + 2.4)];
        right  = [x + robot_size*cos(th - 2.4),  y + robot_size*sin(th - 2.4)];

        robot_marker = patch([front(1), left(1), right(1)], ...
                             [front(2), left(2), right(2)], ...
                             cR, 'FaceAlpha', 0.9, 'EdgeColor', 'k', 'LineWidth', 2);

        % draw heading arrow
        arrow_len = robot_size * 1.5;
        robot_arrow = plot([x, x + arrow_len*cos(th)], ...
                           [y, y + arrow_len*sin(th)], ...
                           'k-', 'LineWidth', 2.5);

        % update title with current step info
        spd = sqrt(vel_x(k)^2 + vel_y(k)^2);
        title(sprintf('Step %d / %d  |  Time = %.0f s  |  Speed = %.1f m/s (%.0f km/h)', ...
                      k, N, data.time(k), spd, spd*3.6), 'FontSize', 11);

        drawnow;
        pause(0.03);
    end

    fprintf('Navigation demo finished.\n');
end
