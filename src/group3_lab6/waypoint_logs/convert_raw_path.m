close all;
clear all;

csv_data = load('wp-2020-05-14-07-25-57.csv');
csv_data = csv_data(13:end-38,:);
csv_data = [csv_data(2:end,:);csv_data(1,:)];
x = csv_data(:,1);
y = csv_data(:,2);
t = [0:length(x)-1] ./ (length(x)-1);

xy = [x y];

tempPoses = [xy;xy(1,:)];
tempVec = (tempPoses(2:end,1:2)-tempPoses (1:end-1,1:2));
phi = rad2deg(cart2pol(tempVec(:,1), tempVec(:,2)));

r = sqrt(sum((tempVec.^2),2)) / 2;
[u,v]=pol2cart(deg2rad(phi), r);

figure(2)
daspect([1 1 1])
scatter(xy(:,1), xy(:,2))
hold on
daspect([1 1 1])
arrow3(xy(:,1:2),xy(:,1:2)+[u,v],'-r',r/2)

writematrix(csv_data, 'recorded_trajectory.csv');