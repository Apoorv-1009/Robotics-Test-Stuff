function ie_controller(x_des)
% Function that utilises an Integrative Error controller for velocity 
% control of a mathematical model of a car. 
% Integrative error uses a simple Integrative controller but just varies Ki
% gain value proportionally with error.
% State: velocity (x)
% Input: gas/brake (u)   F = c*u 
% Dynamics: x_dot = (c/m)*u - f*x 
% where c is the electro-mechanical transmission coefficient
%       f is friction coefficient 
%       m is mass of the car
%       noise is simulated signal interference

% Initially let velocity be 0
x = 0;
x_dot = 0;
e_int = 0;


% Define constants:
c = 0.8;
m = 1;
f = 0.2;
kie = 1;
ki = 0.1;
dt = 0.1;
gain = 1;
% iterations*dt = seconds of runtime
iterations = 50;


% Define vectors for plotting
velocity = zeros(iterations);
time = zeros(iterations);
input = zeros(iterations);
e = zeros(iterations);
acc = zeros(iterations);

i = 1;
while i <= iterations
    velocity(i) = x;
    
    % Error
    error = x_des - x;
    e_int = e_int + error*dt;
    kie = gain*error;
    e(i) = kie;
    
    % Calculate control input u using proportional logic
    u = kie*e_int + ki*e_int;
    input(i) = u;
    
    % Dynamics
    x_dot = (c/m)*u - f*x;
    acc(i) = x_dot;
    
    % v = u + a*t
    x = x + x_dot*dt;
    
    time(i) = i*dt;
    i=i+1;
end

% Create subplots 
r=3; c=2;
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


end