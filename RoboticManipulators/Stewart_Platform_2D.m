function Stewart_Platform_2D(coordinates, theta)
% Function that returns link lengths for given coordinates and orientation
% for a 2D stewart platform
% Input format: Stewart_2D([desired_x; desired_y]), desired_theta_in_degrees)
% Link1: black to magenta     Link2: black to cyan

% Convert input angle to radian
theta = (theta*pi)/180;  

% Define position of link bases
a1 = [-2.5; 0];
a2 = [2.5; 0];

% Define link length attachment to the platform in the platform frame
b1 = [-1; 0];
b2 = [1; 0];

% Define rotation matrix
R = [cos(theta), -sin(theta); 
     sin(theta), cos(theta)];

% Calculate link positions
s1 = coordinates + R*b1 - a1;
s2 = coordinates + R*b2 - a2;

% Calculate link lengths
link1_length = norm(s1)
link2_length = norm(s2)

x_vals = [ a1(1), s1(1)+a1(1), s2(1)+a2(1), a2(1) ];
y_vals = [ a1(2), s1(2)+a1(2), s2(2)+a2(2), a2(2) ];

width = 15;

clf
hold on
plot(x_vals, y_vals, 'b','LineWidth',width/5);   %Plot lines

plot(a1(1), a1(2),'ks','LineWidth',width);   % 1st base of platform position
plot(a2(1), a2(2),'ks','LineWidth',width);   % 2nd base of platform position

plot(s1(1)+a1(1), s1(2)+a1(2),'ms','LineWidth',width);   % Link 1 attachment to the platform position
plot(s2(1)+a2(1), s2(2)+a2(2),'cs','LineWidth',width);   % Link 2 attachment to the platform position

plot(coordinates(1), coordinates(2),'gs','LineWidth',width);   % Centre of platform position
hold off

end 
