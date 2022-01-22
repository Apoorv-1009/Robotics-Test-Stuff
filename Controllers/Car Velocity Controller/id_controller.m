function id_controller(x_des)
% Function that utilises an Integral Derivative controller for velocity control of a
% mathematical model of a car. 
% State: velocity (x)
% Input: gas/brake (u)   F = c*u 
% Dynamics: x_dot = (c/m)*u - f*x
% where c is the electro-mechanical transmission coefficient
%       f is friction coefficient 
%       m is mass of the car

% Initially let velocity be 0
x = 0;
x_dot = 0;
x_prev = 0;
e_int = 0;

% Define constants:
c = 0.8;
m = 1;
f = 0.2;
ki = 0.02;
kd = 0.01;
dt = 0.1;
% iterations*dt = seconds of runtime
iterations = 700;


% Define vectors for plotting
velocity = zeros(iterations);
time = zeros(iterations);
input = zeros(iterations);
e = zeros(iterations);
integral = zeros(iterations);
derivative = zeros(iterations);
acc = zeros(iterations);

i = 1;
while i <= iterations
    velocity(i) = x;
    
    % Error
    error = x_des - x;
    e_int = e_int + error*dt;
    e_dot = -(x - x_prev)/dt;
    e(i) = error;
    
    % Calculate control input u using proportional logic
    u = ki*e_int + kd*e_dot;
    input(i) = u;
    integral(i) = ki*e_int;
    derivative(i) = kd*e_dot;
    
    % Dynamics
    x_dot = (c/m)*u - f*x;
    acc(i) = x_dot;
        
    x_prev = x; 
    % v = u + a*t
    x = x + x_dot*dt;
    
    time(i) = i*dt;
    i=i+1;
end

% Create subplots 

% Create subplots 
r=2; c=3;
% Plot velocity vs time
subplot(r,c,1)
plot(time, velocity)
ylabel('velocity')
xlabel('time')
title('velocity vs time')
grid on

% Plot u vs time
subplot(r,c,2)
plot(time, input)
ylabel('u')
xlabel('time')
title('u vs time')
grid on

% Plot error  vs time
subplot(r,c,3)
plot(time, e)
ylabel('error')
xlabel('time')
title('error vs time')
grid on

% Plot acceleration  vs time
subplot(r,c,4)
plot(time, acc)
ylabel('accleration')
xlabel('time')
title('acceleration vs time')
grid on

% Plot proportional integral and derivative input vs time
subplot(r, c, 5)
% Integral output is green
% Derivative output in red
plot(time, integral, 'g', time, derivative, 'r')
title('ID vs time')
grid on

% Print final velocity achieved
disp(velocity(iterations))

end