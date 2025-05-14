%% Mobile Robot Kinematics and Dynamics Simulation with Payload Changes and Improved Controller
% This script simulates a differential drive mobile robot with both
% kinematics and dynamics models, including payload pickup and dropoff.

clear all;
close all;
clc;

%% Robot Parameters
% Define robot physical parameters
robot.wheel_radius = 0.05;  % wheel radius [m]
robot.wheel_base = 0.2;     % distance between wheels [m]
robot.mass = 1.0;           % robot mass [kg]
robot.I = 0.1;              % moment of inertia [kg*m^2]
robot.body_length = 0.3;    % robot body length [m]
robot.body_width = 0.2;     % robot body width [m]

% Friction parameters
robot.B_r = 0.01;  % viscous friction coefficient for right wheel
robot.B_l = 0.01;  % viscous friction coefficient for left wheel

% Calculate dynamic model parameters
robot.J_r = (robot.mass * robot.wheel_radius^2 / 4) + ...
           (robot.I * robot.wheel_radius^2 / robot.wheel_base^2);
robot.J_l = robot.J_r;
robot.J_rl = (robot.mass * robot.wheel_radius^2 / 4) - ...
            (robot.I * robot.wheel_radius^2 / robot.wheel_base^2);

%% Simulation Parameters
dt = 0.05;              % time step [s]
sim_time = 10;          % simulation time [s]
num_steps = sim_time/dt;  % number of time steps

%% Define the Figure-8 Path & Payload Changes
% Common setup for both controllers

% Define figure-8 path
t_values = linspace(0, 2*pi, num_steps+1);
scale_x = 2.0;
scale_y = 1.0;
path_x = scale_x * sin(t_values);
path_y = scale_y * sin(t_values) .* cos(t_values);

% Define payload change points
pickup_point = round(num_steps/4);     % Pick up payload at 1/4 of the path
dropoff_point = round(3*num_steps/4);  % Drop off payload at 3/4 of the path
payload_mass = 2.0;             % Additional mass [kg]
payload_inertia = 0.2;          % Additional moment of inertia [kg*m^2]

%% Scenario 2A: Figure-8 Path Following with Standard Controller

% Reset robot state and parameters
robot_standard = robot;  % Start with original parameters
x = 0;
y = 0;
theta = 0;
omega_r = 0;
omega_l = 0;

% Initialize history arrays for standard controller
history_standard.x = zeros(1, num_steps+1);
history_standard.y = zeros(1, num_steps+1);
history_standard.theta = zeros(1, num_steps+1);
history_standard.v = zeros(1, num_steps+1);
history_standard.omega = zeros(1, num_steps+1);
history_standard.time = zeros(1, num_steps+1);
history_standard.mass = zeros(1, num_steps+1); % Track mass changes
history_standard.x(1) = x;
history_standard.y(1) = y;
history_standard.theta(1) = theta;
history_standard.time(1) = 0;
history_standard.mass(1) = robot_standard.mass;

% Standard controller gains
kp_v = 2.0;     % Proportional gain for velocity
kp_omega = 5.0; % Proportional gain for angular velocity

% Main simulation loop for standard controller
for i = 1:num_steps
    % Current time
    t = (i-1) * dt;
    
    % Current position and orientation
    current_x = x;
    current_y = y;
    current_theta = theta;
    
    % Check for payload changes
    if i == pickup_point
        % Pick up payload - increase mass and inertia
        robot_standard.mass = robot_standard.mass + payload_mass;
        robot_standard.I = robot_standard.I + payload_inertia;
        
        % Recalculate dynamic parameters
        robot_standard.J_r = (robot_standard.mass * robot_standard.wheel_radius^2 / 4) + ...
                   (robot_standard.I * robot_standard.wheel_radius^2 / robot_standard.wheel_base^2);
        robot_standard.J_l = robot_standard.J_r;
        robot_standard.J_rl = (robot_standard.mass * robot_standard.wheel_radius^2 / 4) - ...
                    (robot_standard.I * robot_standard.wheel_radius^2 / robot_standard.wheel_base^2);
        
        fprintf('Standard Controller: Payload picked up at step %d. New mass: %.1f kg\n', i, robot_standard.mass);
    elseif i == dropoff_point
        % Drop off payload - return to original parameters
        robot_standard.mass = robot_standard.mass - payload_mass;
        robot_standard.I = robot_standard.I - payload_inertia;
        
        % Recalculate dynamic parameters
        robot_standard.J_r = (robot_standard.mass * robot_standard.wheel_radius^2 / 4) + ...
                   (robot_standard.I * robot_standard.wheel_radius^2 / robot_standard.wheel_base^2);
        robot_standard.J_l = robot_standard.J_r;
        robot_standard.J_rl = (robot_standard.mass * robot_standard.wheel_radius^2 / 4) - ...
                    (robot_standard.I * robot_standard.wheel_radius^2 / robot_standard.wheel_base^2);
        
        fprintf('Standard Controller: Payload dropped off at step %d. New mass: %.1f kg\n', i, robot_standard.mass);
    end
    
    % Target point on the path
    target_x = path_x(i);
    target_y = path_y(i);
    
    % Calculate distance and heading to target
    dx = target_x - current_x;
    dy = target_y - current_y;
    distance = sqrt(dx^2 + dy^2);
    
    % Calculate target heading (direction to the next point)
    if i < num_steps
        % Look ahead for smoother path
        target_heading = atan2(path_y(i+1) - path_y(i), path_x(i+1) - path_x(i));
    else
        target_heading = atan2(dy, dx);
    end
    
    % Calculate heading error
    heading_error = target_heading - current_theta;
    
    % Normalize heading error to [-pi, pi]
    heading_error = atan2(sin(heading_error), cos(heading_error));
    
    % Standard controller (fixed gains)
    % Calculate control inputs using PID
    v = kp_v * distance;  % Linear velocity proportional to distance
    omega = kp_omega * heading_error;  % Angular velocity proportional to heading error
    
    % Limit the control inputs
    v = max(min(v, 1.0), -1.0);  % Limit to [-1, 1]
    omega = max(min(omega, 2.0), -2.0);  % Limit to [-2, 2]
    
    % Convert to wheel velocities
    [omega_l, omega_r] = inverse_kinematics(v, omega, robot_standard);
    
    % Convert wheel velocities to torques (simplified)
    tau_l = omega_l * 0.1;  % Simple gain
    tau_r = omega_r * 0.1;  % Simple gain
    
    % Update dynamics
    [omega_r, omega_l] = update_dynamics(omega_r, omega_l, tau_r, tau_l, robot_standard, dt);
    
    % Forward kinematics to get actual robot velocities
    [v, omega] = forward_kinematics(omega_r, omega_l, robot_standard);
    
    % Update kinematics
    [x, y, theta] = update_kinematics(x, y, theta, v, omega, dt);
    
    % Store states in history
    history_standard.x(i+1) = x;
    history_standard.y(i+1) = y;
    history_standard.theta(i+1) = theta;
    history_standard.v(i+1) = v;
    history_standard.omega(i+1) = omega;
    history_standard.time(i+1) = t + dt;
    history_standard.mass(i+1) = robot_standard.mass;
end

%% Scenario 2B: Figure-8 Path Following with Improved Adaptive Controller

% Reset robot state and parameters
robot_adaptive = robot;  % Start with original parameters
x = 0;
y = 0;
theta = 0;
omega_r = 0;
omega_l = 0;

% Initialize history arrays for adaptive controller
history_adaptive.x = zeros(1, num_steps+1);
history_adaptive.y = zeros(1, num_steps+1);
history_adaptive.theta = zeros(1, num_steps+1);
history_adaptive.v = zeros(1, num_steps+1);
history_adaptive.omega = zeros(1, num_steps+1);
history_adaptive.time = zeros(1, num_steps+1);
history_adaptive.mass = zeros(1, num_steps+1); % Track mass changes
history_adaptive.kp_v = zeros(1, num_steps+1);  % Track controller gains
history_adaptive.kp_omega = zeros(1, num_steps+1);
history_adaptive.x(1) = x;
history_adaptive.y(1) = y;
history_adaptive.theta(1) = theta;
history_adaptive.time(1) = 0;
history_adaptive.mass(1) = robot_adaptive.mass;

% Base controller gains (same as standard controller)
kp_v_base = 2.0;     % Base proportional gain for velocity
kp_omega_base = 5.0; % Base proportional gain for angular velocity
history_adaptive.kp_v(1) = kp_v_base;
history_adaptive.kp_omega(1) = kp_omega_base;

% Adaptive control parameters
ref_mass = robot.mass;  % Reference mass for gain scaling
prev_error = 0;  % For error trend detection

% Main simulation loop for adaptive controller
for i = 1:num_steps
    % Current time
    t = (i-1) * dt;
    
    % Current position and orientation
    current_x = x;
    current_y = y;
    current_theta = theta;
    
    % Check for payload changes
    if i == pickup_point
        % Pick up payload - increase mass and inertia
        robot_adaptive.mass = robot_adaptive.mass + payload_mass;
        robot_adaptive.I = robot_adaptive.I + payload_inertia;
        
        % Recalculate dynamic parameters
        robot_adaptive.J_r = (robot_adaptive.mass * robot_adaptive.wheel_radius^2 / 4) + ...
                   (robot_adaptive.I * robot_adaptive.wheel_radius^2 / robot_adaptive.wheel_base^2);
        robot_adaptive.J_l = robot_adaptive.J_r;
        robot_adaptive.J_rl = (robot_adaptive.mass * robot_adaptive.wheel_radius^2 / 4) - ...
                    (robot_adaptive.I * robot_adaptive.wheel_radius^2 / robot_adaptive.wheel_base^2);
        
        fprintf('Adaptive Controller: Payload picked up at step %d. New mass: %.1f kg\n', i, robot_adaptive.mass);
    elseif i == dropoff_point
        % Drop off payload - return to original parameters
        robot_adaptive.mass = robot_adaptive.mass - payload_mass;
        robot_adaptive.I = robot_adaptive.I - payload_inertia;
        
        % Recalculate dynamic parameters
        robot_adaptive.J_r = (robot_adaptive.mass * robot_adaptive.wheel_radius^2 / 4) + ...
                   (robot_adaptive.I * robot_adaptive.wheel_radius^2 / robot_adaptive.wheel_base^2);
        robot_adaptive.J_l = robot_adaptive.J_r;
        robot_adaptive.J_rl = (robot_adaptive.mass * robot_adaptive.wheel_radius^2 / 4) - ...
                    (robot_adaptive.I * robot_adaptive.wheel_radius^2 / robot_adaptive.wheel_base^2);
        
        fprintf('Adaptive Controller: Payload dropped off at step %d. New mass: %.1f kg\n', i, robot_adaptive.mass);
    end
    
    % Target point on the path
    target_x = path_x(i);
    target_y = path_y(i);
    
    % Calculate distance and heading to target
    dx = target_x - current_x;
    dy = target_y - current_y;
    distance = sqrt(dx^2 + dy^2);
    
    % Calculate target heading (direction to the next point)
    if i < num_steps
        % Look ahead for smoother path
        target_heading = atan2(path_y(i+1) - path_y(i), path_x(i+1) - path_x(i));
    else
        target_heading = atan2(dy, dx);
    end
    
    % Calculate heading error
    heading_error = target_heading - current_theta;
    
    % Normalize heading error to [-pi, pi]
    heading_error = atan2(sin(heading_error), cos(heading_error));
    
    % Calculate error trend
    error_trend = distance - prev_error;
    prev_error = distance;
    
    % IMPROVED CONTROLLER: Adaptive gains based on payload mass and error trend
    
    % 1. Mass-based gain scaling
    mass_ratio = robot_adaptive.mass / ref_mass;
    
    % Scale gains based on mass (higher mass needs higher gains)
    kp_v = kp_v_base * mass_ratio;
    kp_omega = kp_omega_base * mass_ratio;
    
    % 2. Error trend detection
    if error_trend > 0
        % If error is growing, increase gains further
        kp_v = kp_v * (1 + error_trend);
        kp_omega = kp_omega * (1 + error_trend);
    end
    
    % 3. Error-based gain adaptation
    if distance > 0.1
        % If error is large, add additional gain boost
        error_factor = 1 + distance;
        kp_v = kp_v * min(error_factor, 2.0);  % Limit gain increase
        kp_omega = kp_omega * min(error_factor, 2.0);
    end
    
    % Store the adaptive gains
    history_adaptive.kp_v(i+1) = kp_v;
    history_adaptive.kp_omega(i+1) = kp_omega;
    
    % Calculate control inputs using adaptive gains
    v = kp_v * distance;  % Linear velocity proportional to distance
    omega = kp_omega * heading_error;  % Angular velocity proportional to heading error
    
    % Limit the control inputs
    v = max(min(v, 1.0), -1.0);  % Limit to [-1, 1]
    omega = max(min(omega, 2.0), -2.0);  % Limit to [-2, 2]
    
    % Convert to wheel velocities
    [omega_l, omega_r] = inverse_kinematics(v, omega, robot_adaptive);
    
    % Convert wheel velocities to torques with inertia compensation
    tau_l = omega_l * 0.1;  % Base gain
    tau_r = omega_r * 0.1;  % Base gain
    
    % Add inertia compensation (higher torque for higher mass)
    inertia_compensation = 0.05 * (mass_ratio - 1); 
    if mass_ratio > 1
        tau_l = tau_l * (1 + inertia_compensation);
        tau_r = tau_r * (1 + inertia_compensation);
    end
    
    % Update dynamics
    [omega_r, omega_l] = update_dynamics(omega_r, omega_l, tau_r, tau_l, robot_adaptive, dt);
    
    % Forward kinematics to get actual robot velocities
    [v, omega] = forward_kinematics(omega_r, omega_l, robot_adaptive);
    
    % Update kinematics
    [x, y, theta] = update_kinematics(x, y, theta, v, omega, dt);
    
    % Store states in history
    history_adaptive.x(i+1) = x;
    history_adaptive.y(i+1) = y;
    history_adaptive.theta(i+1) = theta;
    history_adaptive.v(i+1) = v;
    history_adaptive.omega(i+1) = omega;
    history_adaptive.time(i+1) = t + dt;
    history_adaptive.mass(i+1) = robot_adaptive.mass;
end

%% Calculate Performance Metrics

% Calculate path errors
errors_standard = zeros(size(history_standard.x));
errors_adaptive = zeros(size(history_adaptive.x));

for i = 1:length(history_standard.x)
    dx_std = history_standard.x(i) - path_x(i);
    dy_std = history_standard.y(i) - path_y(i);
    errors_standard(i) = sqrt(dx_std^2 + dy_std^2);
    
    dx_adp = history_adaptive.x(i) - path_x(i);
    dy_adp = history_adaptive.y(i) - path_y(i);
    errors_adaptive(i) = sqrt(dx_adp^2 + dy_adp^2);
end

% Calculate performance metrics
avg_error_standard = mean(errors_standard);
max_error_standard = max(errors_standard);
avg_error_adaptive = mean(errors_adaptive);
max_error_adaptive = max(errors_adaptive);

% Calculate error improvement percentage
avg_improvement = (1 - avg_error_adaptive/avg_error_standard) * 100;
max_improvement = (1 - max_error_adaptive/max_error_standard) * 100;

fprintf('\nPerformance Comparison:\n');
fprintf('Standard Controller: Average Error = %.4f m, Maximum Error = %.4f m\n', avg_error_standard, max_error_standard);
fprintf('Adaptive Controller: Average Error = %.4f m, Maximum Error = %.4f m\n', avg_error_adaptive, max_error_adaptive);
fprintf('Improvement: Average Error reduced by %.1f%%, Maximum Error reduced by %.1f%%\n', avg_improvement, max_improvement);

% Calculate pickup and dropoff phase errors
pickup_start = pickup_point;
pickup_end = min(pickup_point + round(num_steps/10), num_steps);
dropoff_start = dropoff_point;
dropoff_end = min(dropoff_point + round(num_steps/10), num_steps);

pickup_error_standard = mean(errors_standard(pickup_start:pickup_end));
pickup_error_adaptive = mean(errors_adaptive(pickup_start:pickup_end));
dropoff_error_standard = mean(errors_standard(dropoff_start:dropoff_end));
dropoff_error_adaptive = mean(errors_adaptive(dropoff_start:dropoff_end));

fprintf('\nPickup Phase Error: Standard = %.4f m, Adaptive = %.4f m (%.1f%% reduction)\n', pickup_error_standard, pickup_error_adaptive, (1 - pickup_error_adaptive/pickup_error_standard) * 100);
fprintf('Dropoff Phase Error: Standard = %.4f m, Adaptive = %.4f m (%.1f%% reduction)\n', dropoff_error_standard, dropoff_error_adaptive, (1 - dropoff_error_adaptive/dropoff_error_standard) * 100);

%% Plots

% 1. Path Following Comparison
figure('Name', 'Path Following Comparison', 'Position', [100, 100, 1000, 600]);

% Plot both paths together
subplot(2, 2, [1, 3]);
plot(path_x, path_y, 'k--', 'LineWidth', 2, 'DisplayName', 'Target Path');
hold on;
plot(history_standard.x, history_standard.y, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Standard Controller');
plot(history_adaptive.x, history_adaptive.y, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Adaptive Controller');
grid on;
axis equal;
xlabel('X [m]');
ylabel('Y [m]');
title('Path Following Comparison');
legend('show', 'Location', 'best');

% Mark payload pickup and dropoff points
plot(history_standard.x(pickup_point), history_standard.y(pickup_point), 'ms', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Pickup Point');
plot(history_standard.x(dropoff_point), history_standard.y(dropoff_point), 'ks', 'MarkerSize', 12, 'LineWidth', 2, 'DisplayName', 'Dropoff Point');

% 2. Error Comparison
subplot(2, 2, 2);
plot(history_standard.time, errors_standard, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Standard Controller');
hold on;
plot(history_adaptive.time, errors_adaptive, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Adaptive Controller');
grid on;
xlabel('Time [s]');
ylabel('Path Error [m]');
title('Path Following Error');
legend('show', 'Location', 'best');

% Add vertical lines to mark payload changes
pickup_time = history_standard.time(pickup_point);
dropoff_time = history_standard.time(dropoff_point);
xline(pickup_time, 'g--', 'Pickup', 'LineWidth', 1.5);
xline(dropoff_time, 'g--', 'Dropoff', 'LineWidth', 1.5);

% 3. Mass and Controller Gains
subplot(2, 2, 4);
yyaxis left;
plot(history_standard.time, history_standard.mass, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Robot Mass');
ylabel('Robot Mass [kg]');
ylim([0, max(history_standard.mass)*1.2]);

yyaxis right;
plot(history_adaptive.time, history_adaptive.kp_v, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Adaptive kp_v');
plot(history_adaptive.time, history_adaptive.kp_omega, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Adaptive kp_\omega');
ylabel('Controller Gains');

grid on;
xlabel('Time [s]');
title('Mass and Controller Gains');
legend('show', 'Location', 'best');

% Add vertical lines to mark payload changes
xline(pickup_time, 'g--', 'Pickup', 'LineWidth', 1.5);
xline(dropoff_time, 'g--', 'Dropoff', 'LineWidth', 1.5);

% 4. Velocity Profiles
figure('Name', 'Velocity Profiles', 'Position', [200, 200, 1000, 400]);

subplot(1, 2, 1);
plot(history_standard.time, history_standard.v, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Standard Controller');
hold on;
plot(history_adaptive.time, history_adaptive.v, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Adaptive Controller');
grid on;
xlabel('Time [s]');
ylabel('Linear Velocity [m/s]');
title('Linear Velocity Profiles');
legend('show', 'Location', 'best');
xline(pickup_time, 'g--', 'Pickup', 'LineWidth', 1.5);
xline(dropoff_time, 'g--', 'Dropoff', 'LineWidth', 1.5);

subplot(1, 2, 2);
plot(history_standard.time, history_standard.omega, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Standard Controller');
hold on;
plot(history_adaptive.time, history_adaptive.omega, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Adaptive Controller');
grid on;
xlabel('Time [s]');
ylabel('Angular Velocity [rad/s]');
title('Angular Velocity Profiles');
legend('show', 'Location', 'best');
xline(pickup_time, 'g--', 'Pickup', 'LineWidth', 1.5);
xline(dropoff_time, 'g--', 'Dropoff', 'LineWidth', 1.5);

% 5. Performance Metrics Bar Chart
figure('Name', 'Performance Metrics', 'Position', [300, 300, 600, 400]);
metrics = [avg_error_standard, avg_error_adaptive; max_error_standard, max_error_adaptive];
bar(metrics);
set(gca, 'XTickLabel', {'Average Error', 'Maximum Error'});
legend('Standard Controller', 'Adaptive Controller', 'Location', 'best');
ylabel('Path Error [m]');
title('Error Reduction with Adaptive Controller');

%% Helper Functions

function [omega_r, omega_l] = update_dynamics(omega_r, omega_l, tau_r, tau_l, robot, dt)
    % Dynamic equations relating torques to wheel accelerations
    % [tau_r]   [J_r  J_rl] [omega_dot_r]
    % [tau_l] = [J_rl J_l ] [omega_dot_l]
    
    % Including friction: tau = J*omega_dot + B*omega
    
    % Compute the determinant of the inertia matrix
    det_J = robot.J_r * robot.J_l - robot.J_rl^2;
    
    % Compute wheel accelerations
    omega_dot_r = ((robot.J_l * (tau_r - robot.B_r * omega_r) - ...
                   robot.J_rl * (tau_l - robot.B_l * omega_l)) / det_J);
               
    omega_dot_l = ((robot.J_r * (tau_l - robot.B_l * omega_l) - ...
                   robot.J_rl * (tau_r - robot.B_r * omega_r)) / det_J);
    
    % Update wheel velocities
    omega_r = omega_r + omega_dot_r * dt;
    omega_l = omega_l + omega_dot_l * dt;
end

function [v, omega] = forward_kinematics(omega_r, omega_l, robot)
    % Linear and angular velocity of the robot
    v = robot.wheel_radius * (omega_r + omega_l) / 2;
    omega = robot.wheel_radius * (omega_r - omega_l) / robot.wheel_base;
end

function [omega_l, omega_r] = inverse_kinematics(v, omega, robot)
    % Angular velocities of the wheels
    omega_l = (2 * v - omega * robot.wheel_base) / (2 * robot.wheel_radius);
    omega_r = (2 * v + omega * robot.wheel_base) / (2 * robot.wheel_radius);
end

function [x, y, theta] = update_kinematics(x, y, theta, v, omega, dt)
    % Update robot position and orientation
    x = x + v * cos(theta) * dt;
    y = y + v * sin(theta) * dt;
    theta = theta + omega * dt;
    
    % Normalize theta to [-pi, pi]
    theta = atan2(sin(theta), cos(theta));
end