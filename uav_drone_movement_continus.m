clc; clear;

%% PARAMETERS
scale = 1e5;
lat = [21.24987208329769, 21.25807802569907];
lon = [81.60473512913642, 81.57956938003011];
start = [lat(1)*scale, lon(1)*scale, 30];
goal  = [lat(2)*scale, lon(2)*scale, 30];
goalGround = [goal(1:2), 0];

rng(42); numObs = 20;
obsLat = linspace(lat(1), lat(2), numObs)' + 0.00015 * randn(numObs,1);
obsLon = linspace(lon(1), lon(2), numObs)' + 0.00015 * randn(numObs,1);
obsAlt = 20 + 30 * rand(numObs,1);
obstacles = [obsLat*scale, obsLon*scale, obsAlt];
obsRadius = 60;

%% RRT* PATH
stepSize = 100; maxIter = 3000; goalRadius = 80;
nodes(1).pos = start; nodes(1).parent = 0; nodes(1).cost = 0;
goalReached = false; goalIdx = -1;

for iter = 1:maxIter
    if rand < 0.3
        q_rand = goal;
    else
        x = min(start(1),goal(1)) + rand * abs(diff([start(1),goal(1)]));
        y = min(start(2),goal(2)) + rand * abs(diff([start(2),goal(2)]));
        z = 20 + 20 * rand;
        q_rand = [x, y, z];
    end

    allNodes = reshape([nodes.pos],3,[])';
    [~, idx] = min(vecnorm(allNodes - q_rand, 2, 2));
    q_near = nodes(idx).pos;

    dir = (q_rand - q_near) / norm(q_rand - q_near);
    q_new = q_near + stepSize * dir;

    if ~collisionCheck(q_new, obstacles, obsRadius)
        newNode.pos = q_new;
        newNode.parent = idx;
        newNode.cost = nodes(idx).cost + norm(q_new - q_near);
        nodes(end+1) = newNode;

        if norm(q_new - goal) < goalRadius
            goalReached = true;
            goalIdx = length(nodes);
            break;
        end
    end
end

if ~goalReached
    error('❌ Goal not reached by RRT*.');
end

%% BACKTRACK RRT PATH
rrt_path = [];
newNode.pos = goal;
newNode.parent = goalIdx;
newNode.cost = nodes(goalIdx).cost + norm(goal - nodes(goalIdx).pos);
nodes(end+1) = newNode;

idx = length(nodes);
while idx > 0
    rrt_path = [nodes(idx).pos; rrt_path];
    idx = nodes(idx).parent;
end
rrt_path = [rrt_path; goal(1:2), 0];

%% SMOOTH RRT*
smooth_path = [];
for i = 1:size(rrt_path,1)-1
    p0 = rrt_path(i,:); p1 = rrt_path(i+1,:);
    t = linspace(0,1,20)';
    a0 = p0;
    a3 = 10*(p1-p0);
    a4 = -15*(p1-p0);
    a5 = 6*(p1-p0);
    segment = a0 + a3.*t.^3 + a4.*t.^4 + a5.*t.^5;
    smooth_path = [smooth_path; segment];
end

%% APF ON SMOOTHED PATH
stepSize = 50; repRadius = 60; k_att = 1.5; k_rep = 5000;
threshold = 30; maxSteps = 1000;

pos = smooth_path(1,:);
path_apf = pos;

for i = 1:maxSteps
    F_att = k_att * (goal - pos);
    F_rep = [0 0 0];

    for j = 1:numObs
        diff = pos - obstacles(j,:);
        dist = norm(diff);
        if dist < repRadius && dist > 1e-3
            F_rep = F_rep + k_rep*((1/dist - 1/repRadius)/dist^2)*(diff/dist);
        end
    end

    F_total = F_att + F_rep;
    if norm(F_total) < 1e-3, break; end

    pos = pos + stepSize * F_total / norm(F_total);
    path_apf = [path_apf; pos];

    if norm(pos - goal) < threshold, break; end
end

path_apf = [path_apf; goal; goalGround];

%% INFINITE FORWARD + RETURN PATH
forwardPath = path_apf;
backwardPath = flipud(path_apf(2:end-1,:));
full_path = [forwardPath; backwardPath];

lat_full = full_path(:,1)/scale;
lon_full = full_path(:,2)/scale;
alt_full = full_path(:,3);

%% INFINITE ANIMATION (UNTIL USER CLOSES FIGURE)
fig = figure('Name','Infinite UAV Animation','Color','w');
gx = geoaxes(fig);
geobasemap(gx,'satellite'); hold(gx,'on');

geoscatter(gx, lat(1), lon(1), 100, 'g', 'filled');
geoscatter(gx, lat(2), lon(2), 100, 'r', 'filled');

title(gx,'UAV Continuous Mission (Start ⇄ Goal)','FontSize',14)

N = length(lat_full);
i = 1; direction = 1;

while isvalid(fig)
    droneSize = 40 + 0.05*alt_full(i);
    color = [0 0.6 1];

    drone = geoscatter(gx, lat_full(i), lon_full(i), droneSize, color, 'filled');
    geoplot(gx, lat_full(1:i), lon_full(1:i), 'c-', 'LineWidth',1.4);

    pause(0.02)

    if isvalid(drone)
        delete(drone)
    end

    i = i + direction;

    if i >= N
        direction = -1;
        i = N;
    elseif i <= 1
        direction = 1;
        i = 1;
    end
end

%% COLLISION CHECK FUNCTION
function flag = collisionCheck(pt, obs, rad)
    flag = any(vecnorm(obs - pt, 2, 2) < rad);
end
