function d_controller(x_des)
% Function that utilises a Derivative controller for velocity control of a
% mathematical model of a car. 
% State: velocity (x)
% Input: gas/brake (u)   F = c*u 
% Dynamics: x_dot = (c/m)*u - f*x
% where c is the electro-mechanical transmission coefficient
%       k is friction coefficient 
%       m is mass of the car

% Initially let velocity be 0
x = 0;
x_dot = 0;

% Define constants:
c = 0.8;
m = 1;
f = 0.2;
kd = 0.1;
dt = 0.01;
error_prev = 0;
e_dot = 0;
% Placing iterations as 10 gives 1 second runtime
iterations = 10;


% Define vectors for plotting
velocity = zeros(iterations);
time = zeros(iterations);
input = zeros(iterations);
e = zeros(iterations);

i = 1;
while i <= iterations
    velocity(i) = x;
    
    % Calculate error
    error = x_des - x;
    e_dot = (error - error_prev)/dt;
    e(i) = error;
    
    % Calculate control input u using proportional logic
    u = kd*e_dot;
    input(i) = u;
    
    % Dynamics
    x_dot = (c/m)*u - f*x;

    % v = u + a*t
    x = x + x_dot*dt;
    
    error_prev = error;
    time(i) = i*dt;
    i=i+1;
end

% Create subplots 

% Plot velocity vs time
subplot(2,2,1)
plot(time, velocity)
% ylim([0, x_des+3])
ylabel('velocity')
% xlim([0, dt*iterations])
xlabel('time')
title('velocity vs time')
grid on

% Plot u vs time
subplot(2,2,2)
plot(time, input)
% ylim([0, x_des+3])
ylabel('u')
% xlim([0, dt*iterations])
xlabel('time')
title('u vs time')
grid on

% Plot error  vs time
subplot(2,2,3)
plot(time, e)
ylim([0, x_des+3])
ylabel('error')
xlim([0, dt*iterations])
xlabel('time')
title('error vs time')
grid on

end