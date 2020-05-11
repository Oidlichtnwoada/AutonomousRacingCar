close all;
clear all;
theta_deg = [0:360];
f = [2, 3, 5, 13];
s = rand(size(f))*pi;
rho = zeros(1,length(theta_deg));
for i=1:length(f)
    rho = rho + abs(sin(s(i) + deg2rad(theta_deg).*f(i)));
end
[x,y] = pol2cart(deg2rad(theta_deg),rho);
plot(x,y)
axis equal