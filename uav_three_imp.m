clc; clear; close all;

%% 1. Define Environment (Map Size and Buildings)
map_size = [200, 200, 150]; % X, Y, Z dimensions

% Generate terrain (flat with slight elevation variations)
[X, Y] = meshgrid(0:5:map_size(1), 0:5:map_size(2));
Z = -10 + 5 * sin(0.05 * X) .* cos(0.05 * Y); % Slight terrain variation

% Define buildings (rectangular obstacles)
num_buildings = 6;
building_positions = randi([30, 170], num_buildings, 2); % Random (X, Y) locations
building_sizes = randi([30, 50], num_buildings, 1); % Random heights
building_widths = randi([20, 40], num_buildings, 1); % Random widths
building_depths = randi([20, 40], num_buildings, 1); % Random depths

%% 2. Define UAV Path (Waypoints)
waypoints = [10, 10, 10; 80, 50, 60; 130, 120, 80; 180, 180, 100];

% Interpolate using spline for a smooth path
t = linspace(0, 1, size(waypoints,1)); % Parameter for waypoints
tt = linspace(0, 1, 100); % Fine-grained interpolation
spline_x = spline(t, waypoints(:,1), tt);
spline_y = spline(t, waypoints(:,2), tt);
spline_z = spline(t, waypoints(:,3), tt);

%% 3. Plot Environment
figure; hold on;
surf(X, Y, Z, 'EdgeColor', 'none'); % Terrain surface
colormap('parula'); % Dark blue terrain
shading interp;

%% 4. Plot Buildings (Cuboids)
for i = 1:num_buildings
    bx = building_positions(i, 1);
    by = building_positions(i, 2);
    base_height = interp2(X, Y, Z, bx, by); % Adjust to terrain height
    top_height = base_height + building_sizes(i);

    % Define cuboid (building) corners
    x_corners = [bx, bx+building_widths(i), bx+building_widths(i), bx, bx];
    y_corners = [by, by, by+building_depths(i), by+building_depths(i), by];
    z_corners = [base_height, base_height, base_height, base_height, base_height];

    % Top face
    fill3(x_corners, y_corners, top_height * ones(1,5), 'y', 'FaceAlpha', 0.8, 'EdgeColor', 'k');

    % Side faces
    for j = 1:4
        fill3([x_corners(j), x_corners(j+1), x_corners(j+1), x_corners(j)], ...
              [y_corners(j), y_corners(j+1), y_corners(j+1), y_corners(j)], ...
              [base_height, base_height, top_height, top_height], ...
              'y', 'FaceAlpha', 0.7, 'EdgeColor', 'k');
    end
end

%% 5. Plot UAV Path
plot3(spline_x, spline_y, spline_z, '-r', 'LineWidth', 2); % Smooth UAV path
scatter3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 50, 'g', 'filled'); % Via Points
scatter3(waypoints(1,1), waypoints(1,2), waypoints(1,3), 100, 'g', 'filled', 'o'); % Start Point
scatter3(waypoints(end,1), waypoints(end,2), waypoints(end,3), 100, 'r', 'filled', 'p'); % Goal Point

%% 6. Labels and View
xlabel('X [meters]'); ylabel('Y [meters]'); zlabel('Z [meters]');
title('3D UAV Spline Path with Buildings');
legend('Simulated', 'Start', 'Goal');
view(30, 30); % Set viewing angle
grid on; hold off;
