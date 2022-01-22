function pd_controller(x_des)
% Function that utilises a Proportional Derivative controller for velocity control of a
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
x_prev = x;

% Define constants:
c = 0.8;
m = 1;
f = 0.2;
kp = 5;
kd = 0.1;
dt = 0.1;
% iterations*dt = seconds of runtime
iterations = 50;


% Define vectors for plotting
velocity = zeros(iterations);
time = zeros(iterations);
input = zeros(iterations);
e = zeros(iterations);
proportional = zeros(iterations);
derivative = zeros(iterations);

i = 1;
while i <= iterations
    velocity(i) = x;
    
    % Error
    error = x_des - x;
    % error_dot = x_dot
    e_dot = (x - x_prev)/dt;
    e(i) = error;
    
    % Calculate control input u using proportional logic
    u = kp*error + kd*e_dot;
    input(i) = u;
    proportional(i) = kp*error;
    derivative(i) = abs(kd*e_dot);
    
    % Dynamics
    x_dot = (c/m)*u - f*x;

    x_prev = x;
    % v = u + a*t
    x = x + x_dot*dt;
    
    time(i) = i*dt;
    i=i+1;
end

% Create subplots 

% Plot velocity vs time
subplot(2,2,1)
plot(time, velocity)
ylim([0, x_des+3])
ylabel('velocity')
xlim([0, dt*iterations])
xlabel('time')
title('velocity vs time')
grid on

% Plot u vs time
subplot(2,2,2)
plot(time, input)
ylabel('u')
xlabel('time')
title('u vs time')
grid on

% Plot error  vs time
subplot(2,2,3)
plot(time, e)
ylabel('error')
xlabel('time')
title('error vs time')
grid on

% Plot proportional and derivative input vs time
subplot(2, 2, 4)
% Proportional output is blue
% Derivative output in red
plot(time, proportional, time, derivative, 'r')
title('PD vs time')
grid on

% Print final velocity achieved
disp(velocity(iterations))

end