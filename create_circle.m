function [x,y] = create_circle(points,r)

% CREATE_CIRCLE - Plot cirlce with origin as centre 
%
%
% Inputs:
%    points - number of points in circle
%    r - radius of cirle
%
% Outputs:
%    x - x axis circle coordinates
%    y - y axis circle coordinates


% Define x and y coordinates of circle
theta = linspace(0, 2*pi, points);
x = r*cos(theta);
y = r*sin(theta);

end
