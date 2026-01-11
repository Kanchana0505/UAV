clc; clear; close all;

%% Number of waypoints
n = 100;

%%  Base coordinates (Lat, Lon)
lat = [21.249872, 21.258078];
lon = [81.604735, 81.579569];
lat_path = linspace(lat(1), lat(2), n);
lon_path = linspace(lon(1), lon(2), n);
alt_ideal = linspace(100, 100, n);

 %%  Map zoom limits
latlim = [min(lat_path)-0.001, max(lat_path)+0.001];
lonlim = [min(lon_path)-0.001, max(lon_path)+0.001];

%% Disturbance generation
wind_lat = lat_path + 0.00005*sin(2*pi*(1:n)/n);
wind_lon = lon_path + 0.00005*cos(2*pi*(1:n)/n);

noise_lat = lat_path + 0.00003*randn(1,n);
noise_lon = lon_path + 0.00003*randn(1,n);

alpha = 0.92;
lag_lat = filter(1 - alpha, [1, -alpha], lat_path);
lag_lon = filter(1 - alpha, [1, -alpha], lon_path);

spoof_lat = lat_path; spoof_lon = lon_path;
spoof_lat(40:60) = spoof_lat(40:60) + 0.0008;
spoof_lon(40:60) = spoof_lon(40:60) - 0.0008;

obs_lat = lat_path; obs_lon = lon_path;
obs_lat(45:55) = obs_lat(45:55) + 0.0005*sin(linspace(0, pi, 11));
obs_lon(45:55) = obs_lon(45:55) + 0.0005*cos(linspace(0, pi, 11));

terrain_alt = 100 + 20*sin(2*pi*(1:n)/n);
%% Colors - Satellite-visible and distinct
cyan     = [0 255 255]/255;   % Ideal Path
yellow   = [255 255 0]/255;   % Wind Drift
orange   = [255 165 0]/255;   % Sensor Noise
violet   = [148 0 211]/255;   % Actuator Lag
white    = [1 1 1];           % GPS Spoofing
red      = [255 69 0]/255;    % Obstacle Avoidance
brown    = [139 69 19]/255;   % Terrain Altitude
%% Create figure
figure('Name','UAV Path Disturbances on Real Map','Color','w', 'Position',[100 100 1400 900]);
tiledlayout(3,3,'Padding','compact','TileSpacing','compact');

%% 1. Ideal Path
nexttile;
geoplot(lat_path, lon_path, '-', 'Color', cyan, 'LineWidth', 2); hold on;
geoplot(lat_path(1), lon_path(1), 'go', 'MarkerFaceColor','g');
geoplot(lat_path(end), lon_path(end), 'rs', 'MarkerFaceColor','r');
geolimits(latlim, lonlim); geobasemap satellite;
title('1. Ideal UAV Path','FontWeight','bold');
legend('Ideal Path','Start','Goal','FontSize',8,'Location','southeast');

%% 2. Wind Drift
nexttile;
geoplot(lat_path, lon_path, '--', 'Color', cyan); hold on;
geoplot(wind_lat, wind_lon, '-', 'Color', yellow, 'LineWidth', 1.5);
text(wind_lat(70), wind_lon(70), '← wind','FontSize',7,'Color','w');
geolimits(latlim, lonlim); geobasemap satellite;
title('2. Wind Drift','FontWeight','bold');
legend('Ideal','Wind Drift','FontSize',8,'Location','southeast');

%% 3. Sensor Noise
nexttile;
geoplot(lat_path, lon_path, '--', 'Color', cyan); hold on;
geoplot(noise_lat, noise_lon, '-', 'Color', orange, 'LineWidth', 1.5);
text(noise_lat(60), noise_lon(60), '↗ noise','FontSize',7,'Color','w');
geolimits(latlim, lonlim); geobasemap satellite;
title('3. Sensor Noise','FontWeight','bold');
legend('Ideal','Sensor Noise','FontSize',8,'Location','southeast');

%% 4. Actuator Lag
nexttile;
geoplot(lat_path, lon_path, '--', 'Color', cyan); hold on;
geoplot(lag_lat, lag_lon, '-', 'Color', violet, 'LineWidth', 1.5);
text(lag_lat(80), lag_lon(80), '→ lag','FontSize',7,'Color','w');
geolimits(latlim, lonlim); geobasemap satellite;
title('4. Actuator Lag','FontWeight','bold');
legend('Ideal','Actuator Lag','FontSize',8,'Location','southeast');

%% 5. GPS Spoofing
nexttile;
geoplot(lat_path, lon_path, '--', 'Color', cyan); hold on;
geoplot(spoof_lat, spoof_lon, '-', 'Color', white, 'LineWidth', 1.5);
text(spoof_lat(50), spoof_lon(50), '⬅ spoofed','FontSize',7,'Color','k');
geolimits(latlim, lonlim); geobasemap satellite;
title('5. GPS Spoofing','FontWeight','bold');
legend('Ideal','Spoofed Path','FontSize',8,'Location','southeast');

%% 6. Obstacle Avoidance
nexttile;
geoplot(lat_path, lon_path, '--', 'Color', cyan); hold on;
geoplot(obs_lat, obs_lon, '-', 'Color', red, 'LineWidth', 1.5);
text(obs_lat(52), obs_lon(52), '⤴ avoid','FontSize',7,'Color','w');
geolimits(latlim, lonlim); geobasemap satellite;
title('6. Obstacle Avoidance','FontWeight','bold');
legend('Ideal','Obstacle Avoidance','FontSize',8,'Location','southeast');

%% 7. Terrain Altitude (Simulated)
nexttile;
plot(1:n, alt_ideal, '--', 'Color', cyan); hold on;
plot(1:n, terrain_alt, '-', 'Color', brown, 'LineWidth', 1.5);
xlabel('Waypoint'); ylabel('Altitude (m)');
title('7. Simulated Terrain-Aware Altitude','FontWeight','bold');
legend('Ideal Altitude','Terrain Adjusted','Location','south','FontSize',8); grid on;

%% 8. Combined Overlay
nexttile;
geoplot(lat_path, lon_path, '--', 'Color', cyan); hold on;
geoplot(wind_lat, wind_lon, 'Color', yellow);
geoplot(noise_lat, noise_lon, 'Color', orange);
geoplot(lag_lat, lag_lon, 'Color', violet);
geoplot(spoof_lat, spoof_lon, 'Color', white);
geoplot(obs_lat, obs_lon, 'Color', red);
geolimits(latlim, lonlim); geobasemap satellite;
title('8. Combined View of Disturbances','FontWeight','bold');
legend({'Ideal','Wind','Noise','Lag','Spoofing','Obstacle'},'FontSize',7,'Location','eastoutside');

%% 9. Summary Key
nexttile;
axis off;
text(0.1, 0.95, '🛰️ UAV Disturbance Legend','FontWeight','bold','FontSize',12);
text(0.1, 0.85, '• Cyan — Ideal Path');
text(0.1, 0.78, '• Yellow — Wind Drift');
text(0.1, 0.71, '• Orange — Sensor Noise');
text(0.1, 0.64, '• Violet — Actuator Lag');
text(0.1, 0.57, '• White — GPS Spoofing');
text(0.1, 0.50, '• Red — Obstacle Avoidance');
text(0.1, 0.43, '• Brown — Terrain Altitude');

%% Optional Export
% Uncomment to export high-res output:
% exportgraphics(gcf, 'UAV_Path_Disturbances_HighContrast.png', 'Resolution', 300);
% exportgraphics(gcf, 'UAV_Path_Disturbances_HighContrast.pdf', 'ContentType','vector');
