function p_controller(x_des)
% Function that utilises a Proportional controller for velocity control of a
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

% Define constants:
c = 0.8;
m = 1;
f = 0.2;
kp = 5;
dt = 0.1;
% iterations*dt = seconds of runtime
iterations = 30;


% Define vectors for plotting
velocity = zeros(iterations);
time = zeros(iterations);
input = zeros(iterations);
e = zeros(iterations);

i = 1;
while i <= iterations
    velocity(i) = x;
    
    % Error
    error = x_des - x;
    e(i) = error;
    
    % Calculate control input u using proportional logic
    u = kp*error;
    input(i) = u;
    
    % Dynamics
    x_dot = (c/m)*u - f*x;

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
ylim([0, x_des+3])
ylabel('u')
xlim([0, dt*iterations])
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

% Print final velocity achieved
disp(velocity(iterations))

end





