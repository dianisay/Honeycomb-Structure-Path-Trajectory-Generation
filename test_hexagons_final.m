clear all
close all
% Main
Nx=5;
Ny=5;
hex_side=5;

num_points = 20;
rise=20;
home_pos = [0; 0; rise]; % Home position (mm)

outline_color = [
    0 0 0
];

outline_idx = [
    1 1
    1 2
    1 3
    1 4
    1 5
    2 5
    3 4
    4 4
    5 3
    4 3
    3 2
    2 2
]; 

fill_color = [
    1 0 0
    0 1 0
    0 0 1
];

fill_idx = [
  2 3
  2 4
  3 3
]; 

G = createGrid(Nx, Ny, hex_side);

%------------------Outline Trajectory-------
%Home to first outline hexagon
x = outline_idx(1,1);
y = outline_idx(1,2);
center = squeeze(G(y,x,:))';
pts = hexagonPerimeter(center, hex_side);
target=pts(1,:)';
target(3)=rise;
outline_trajectory=linePoints(home_pos,target,num_points);
new_pos=target;
target(3) = 0;        % lower z
outline_trajectory = [outline_trajectory linePoints(new_pos, target, num_points)];
new_pos = target;
hex = [pts, zeros(size(pts,1),1)]';
outline_trajectory = [outline_trajectory hex];
target(3) = rise;        % rise z
outline_trajectory = [outline_trajectory linePoints(new_pos, target, num_points)];
new_pos=target;
%Loop for all hexagons
for i = 2:size(outline_idx, 1)
    x = outline_idx(i,1);
    y = outline_idx(i,2);
    center = squeeze(G(y,x,:))';
    pts = hexagonPerimeter(center, hex_side);
    target=pts(1,:)';
    target(3)=rise;
    outline_trajectory = [outline_trajectory linePoints(new_pos, target, num_points)];
    new_pos=target;
    target(3) = 0;        % lower z
    outline_trajectory = [outline_trajectory linePoints(new_pos, target, num_points)];
    new_pos = target;
    hex = [pts, zeros(size(pts,1),1)]';
    outline_trajectory = [outline_trajectory hex];
    target(3) = rise;        % rise z
    outline_trajectory = [outline_trajectory linePoints(new_pos, target, num_points)];
    new_pos=target;
end

%------------------Fill Trajectory-------
%Last outline hexagon to first fill hexagon
x = fill_idx(1,1);
y = fill_idx(1,2);
center = squeeze(G(y,x,:))';
pts = hexagonPerimeter(center, hex_side);
target=pts(1,:)';
target(3)=rise;
fill_trajectory=linePoints(new_pos,target,num_points);
new_pos=target;
target(3) = 0;        % lower z
fill_trajectory = [fill_trajectory linePoints(new_pos, target, num_points)];
new_pos = target;
hex = [pts, zeros(size(pts,1),1)]';
fill_trajectory = [fill_trajectory hex];
target(3) = rise;        % rise z
fill_trajectory = [fill_trajectory linePoints(new_pos, target, num_points)];
new_pos=target;
%Loop for all hexagons
for i = 2:size(fill_idx, 1)
    x = fill_idx(i,1);
    y = fill_idx(i,2);
    center = squeeze(G(y,x,:))';
    pts = hexagonPerimeter(center, hex_side);
    target=pts(1,:)';
    target(3)=rise;
    fill_trajectory = [fill_trajectory linePoints(new_pos, target, num_points)];
    new_pos=target;
    target(3) = 0;        % lower z
    fill_trajectory = [fill_trajectory linePoints(new_pos, target, num_points)];
    new_pos = target;
    hex = [pts, zeros(size(pts,1),1)]';
    fill_trajectory = [fill_trajectory hex];
    target(3) = rise;        % rise z
    fill_trajectory = [fill_trajectory linePoints(new_pos, target, num_points)];
    new_pos=target;
end

%------------------ Plot--------------------
figure
% --- Stage 1: Outline | Trajectory ---
subplot(2,2,1)
scatter3(G(:,:,1), G(:,:,2), zeros(size(G(:,:,1))), 6, [0 0 0],'filled')
axis equal
grid on
hold on
for i = 1:size(outline_trajectory, 2)
    scatter3(outline_trajectory(1,i), outline_trajectory(2,i), outline_trajectory(3,i), 6, 'blue', 'filled')
end
title('Stage 1: Outline | Trajectory')
hold off

% --- Stage 1: Outline | Image ---
subplot(2,2,2)
scatter3(G(:,:,1), G(:,:,2), zeros(size(G(:,:,1))), 6, [0 0 0],'filled')
axis equal
grid on
hold on
for i = 1:size(outline_idx, 1)
    x = outline_idx(i,1);
    y = outline_idx(i,2);
    center = squeeze(G(y,x,:))';
    pts = hexagonPerimeter(center, hex_side);
    fill3(pts(:,1), pts(:,2), zeros(size(pts(:,1))), outline_color, 'EdgeColor', 'none');
end
title("Stage 1: Outline | Image")
hold off

% --- Stage 2: Fill | Trajectory ---
subplot(2,2,3)
scatter3(G(:,:,1), G(:,:,2), zeros(size(G(:,:,1))), 6, [0 0 0],'filled')
axis equal
grid on
hold on
for i = 1:size(fill_trajectory, 2)
    scatter3(fill_trajectory(1,i), fill_trajectory(2,i), fill_trajectory(3,i), 6, 'blue', 'filled')
end
title('Stage 2: Fill | Trajectory')
hold off

% --- Stage 2: Fill | Image ---
subplot(2,2,4)
scatter3(G(:,:,1), G(:,:,2), zeros(size(G(:,:,1))), 6, [0 0 0],'filled')
axis equal
grid on
hold on
for i = 1:size(fill_idx, 1)
    x = fill_idx(i,1);
    y = fill_idx(i,2);
    center = squeeze(G(y,x,:))';
    pts = hexagonPerimeter(center, hex_side);
    fill3(pts(:,1), pts(:,2), zeros(size(pts(:,1))), fill_color(i,:), 'EdgeColor', 'none');
end
title('Stage 2: Fill | Image')
hold off

sgtitle('End-Effector Trajectory + Resulting Image (3D)')

%-----------------Functions----------------------
% Create grid
function G = createGrid(Nx, Ny, hex_side)
    xSpacing=1.5*hex_side;
    ySpacing=hex_side*sqrt(3);
    [X, Y] = meshgrid(0:xSpacing:(Nx-1)*xSpacing, 0:ySpacing:(Ny-1)*ySpacing);

    % Shift odd columns by ySpacing/2
    Y(:, 1:2:end) = Y(:, 1:2:end) + ySpacing/2;

    G = zeros(Ny, Nx, 2);
    G(:,:,1) = X;
    G(:,:,2) = Y;
end

% Create Hexagon
function pts = hexagonPerimeter(center, hex_side, n)

    if nargin < 3
        n = 20; %points per edge
    end

    cx = center(1);
    cy = center(2);

    % Radius to vertices
    R = hex_side;

    % Flat-top hexagon vertex angles (degrees)
    angles = [0 60 120 180 240 300] * pi/180;

    % Hexagon vertices
    V = [cx + R*cos(angles)', cy + R*sin(angles)'];

    % Close the polygon
    V = [V; V(1,:)];

    % Generate perimeter points
    pts = [];
    for i = 1:6
        x = linspace(V(i,1), V(i+1,1), n);
        y = linspace(V(i,2), V(i+1,2), n);
        pts = [pts; x(1:end-1)', y(1:end-1)'];
    end
end

%Line between points
function target_trajectory=linePoints(start_pos,end_pos,num_points)
% Create a list of 100 XYZ points between start and end
target_trajectory = [linspace(start_pos(1), end_pos(1), num_points); ...
                     linspace(start_pos(2), end_pos(2), num_points); ...
                     linspace(start_pos(3), end_pos(3), num_points)];
end