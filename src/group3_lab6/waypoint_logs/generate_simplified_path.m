close all;
clear all;

show_figures = true;

csv_data = load('recorded_trajectory.csv');
csv_data = [csv_data(2:end,:);csv_data(1,:)];
x = csv_data(:,1);
y = csv_data(:,2);
t = [0:length(x)-1] ./ (length(x)-1);

if show_figures
    figure(1)
    daspect([1 1 1])
    scatter(x,y)
    axis equal
    title('original recorded trajectory');
end

% =========================================================================
% low-pass filtering

n_harmonics = 1000;
n_freqs = 50;
xy = rEfourier(fEfourier([x y], n_harmonics, false, false), ...
    n_harmonics, n_freqs);
% =========================================================================

waypoints = [xy;xy(1,:)];
segments = waypoints(2:end,1:2) - waypoints(1:end-1,1:2);
yaw = rad2deg(cart2pol(segments(:,1), segments(:,2)));
phi = quaternion([yaw zeros(length(yaw),2)],'eulerd','ZYX','frame');

% compute arrow directions and lengths (just for plotting)
r = sqrt(sum((segments.^2),2)) / 2; % set to half the segment length
[u,v]=pol2cart(deg2rad(yaw), r);

if show_figures
    figure(2)
    daspect([1 1 1])
    scatter(xy(:,1), xy(:,2))
    hold on
    daspect([1 1 1])
    arrow3(xy(:,1:2),xy(:,1:2)+[u,v],'-r',r/2)
    title('low-pass filtered trajectory');
end

writematrix([xy deg2rad(yaw)], 'lowpass_filtered_trajectory.csv');

n = 1000; % desired number of interpolated waypoints

loop_xy = repmat(xy,3,1);
loop_t = [-1:1/length(xy):2];
loop_t = loop_t(1:end-1);
px = pchip(loop_t,loop_xy(:,1),[0:1/n:1])';
py = pchip(loop_t,loop_xy(:,2),[0:1/n:1])';

px = px(1:end-1,:);
py = py(1:end-1,:);

% =========================================================================
new_q = [];
for i=0:length(phi)-1
    j = mod(i+1, length(phi));
    steps = n/length(phi);
    q0 = phi(i+1);
    q1 = phi(j+1);
    q_interp = slerp(q0,q1,[0:1/(steps-1):1]);
    new_q = vertcat(new_q,compact(q_interp));
end

% =========================================================================
new_xy = [px py];
new_waypoints = [new_xy;new_xy(1,:)];
new_segments = new_waypoints(2:end,1:2) - new_waypoints(1:end-1,1:2);

new_yaw = eulerd(quaternion(new_q),'ZYX','frame');
new_yaw = new_yaw(:,1);

new_r = sqrt(sum((new_segments.^2),2)) / 2; % set to half the segment length
[new_u,new_v]=pol2cart(deg2rad(new_yaw), new_r);

if show_figures
    figure(3)
    daspect([1 1 1])
    scatter(new_xy(:,1), new_xy(:,2))
    hold on
    daspect([1 1 1])
    arrow3(new_xy(:,1:2),new_xy(:,1:2)+[new_u,new_v],'-r',new_r/2)
    title('resampled trajectory');
end

writematrix([new_xy deg2rad(new_yaw)], 'resampled_trajectory.csv');
