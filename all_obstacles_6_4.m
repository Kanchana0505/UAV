clc; clear; close all;

% Basic setup
n = 100;
lat = [21.249872, 21.258078];     % start to goal
lon = [81.604735, 81.579569];
alt_ideal = linspace(100, 100, n); % Flat ideal altitude

% Ideal path
lat_path = linspace(lat(1), lat(2), n);
lon_path = linspace(lon(1), lon(2), n);

% Prepare figure
figure('Name','UAV Path Disturbances on Real Map','Color','w','Position',[100 100 1400 900]);
tiledlayout(3,3, 'Padding', 'compact', 'TileSpacing', 'compact');

%% 1. Ideal Path
nexttile;
geoplot(lat_path, lon_path, 'g', 'LineWidth', 2); grid on;
title('1. Ideal Path');
geobasemap streets;

%% 2. Wind Drift
wind_lat = lat_path + 0.00005*sin(2*pi*(1:n)/n);
wind_lon = lon_path + 0.00005*cos(2*pi*(1:n)/n);
nexttile;
geoplot(lat_path, lon_path, '--g'); hold on;
geoplot(wind_lat, wind_lon, 'r', 'LineWidth', 1.5); grid on;
title('2. Wind Drift');
geobasemap streets;

%% 3. Sensor Noise
noise_lat = lat_path + 0.00003*randn(1,n);
noise_lon = lon_path + 0.00003*randn(1,n);
nexttile;
geoplot(lat_path, lon_path, '--g'); hold on;
geoplot(noise_lat, noise_lon, 'm', 'LineWidth', 1.5); grid on;
title('3. Sensor Noise');
geobasemap streets;

%% 4. Actuator Lag
alpha = 0.92;
lag_lat = filter(1 - alpha, [1, -alpha], lat_path);
lag_lon = filter(1 - alpha, [1, -alpha], lon_path);
nexttile;
geoplot(lat_path, lon_path, '--g'); hold on;
geoplot(lag_lat, lag_lon, 'b', 'LineWidth', 1.5); grid on;
title('4. Actuator Lag');
geobasemap streets;

%% 5. GPS Spoofing
spoof_lat = lat_path;
spoof_lon = lon_path;
spoof_lat(40:60) = spoof_lat(40:60) + 0.0008;
spoof_lon(40:60) = spoof_lon(40:60) - 0.0008;
nexttile;
geoplot(lat_path, lon_path, '--g'); hold on;
geoplot(spoof_lat, spoof_lon, 'k', 'LineWidth', 1.5); grid on;
title('5. GPS Spoofing');
geobasemap streets;

%% 6. Dynamic Obstacle Avoidance
obs_lat = lat_path;
obs_lon = lon_path;
obs_lat(45:55) = obs_lat(45:55) + 0.0005*sin(linspace(0, pi, 11));
obs_lon(45:55) = obs_lon(45:55) + 0.0005*cos(linspace(0, pi, 11));
nexttile;
geoplot(lat_path, lon_path, '--g'); hold on;
geoplot(obs_lat, obs_lon, 'c', 'LineWidth', 1.5); grid on;
title('6. Obstacle Avoidance');
geobasemap streets;

%% 7. Terrain Altitude Simulated
terrain_alt = 100 + 20*sin(2*pi*(1:n)/n);
nexttile;
plot(1:n, terrain_alt, '-', 'Color', [1 0.5 0], 'LineWidth', 2);
title('7. Simulated Terrain Altitude');
xlabel('Waypoint'); ylabel('Altitude (m)'); grid on;

%% 8. Combined Path View (All Disturbances)
nexttile;
geoplot(lat_path, lon_path, '--', 'Color', [0.1 0.6 0.1]); hold on;
geoplot(wind_lat, wind_lon, 'r');
geoplot(noise_lat, noise_lon, 'm');
geoplot(lag_lat, lag_lon, 'b');
geoplot(spoof_lat, spoof_lon, 'k');
geoplot(obs_lat, obs_lon, 'c');
legend('Ideal','Wind','Noise','Lag','Spoofing','Obstacle','Location','best');
title('8. All Disturbances Overlay');
geobasemap streets;

%% 9. Disturbance Key
nexttile;
axis off;
text(0.1, 0.9, '🛰️ Disturbance Reference Key', 'FontSize', 12, 'FontWeight','bold');
text(0.1, 0.8, '• Green: Ideal Path');
text(0.1, 0.7, '• Red: Wind Drift');
text(0.1, 0.6, '• Magenta: Sensor Noise');
text(0.1, 0.5, '• Blue: Actuator Lag');
text(0.1, 0.4, '• Black: GPS Spoofing');
text(0.1, 0.3, '• Cyan: Obstacle Avoidance');
text(0.1, 0.2, '• Orange (Plot 7): Terrain Altitude');
