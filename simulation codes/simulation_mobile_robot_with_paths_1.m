%% Mobile Robot Kinematics and Dynamics Simulation
% This script simulates a differential drive mobile robot with both
% kinematics and dynamics models.

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

%% Initialize State and History
% Initial state
x = 0;          % initial x position [m]
y = 0;          % initial y position [m]
theta = 0;      % initial orientation [rad]
omega_r = 0;    % initial right wheel angular velocity [rad/s]
omega_l = 0;    % initial left wheel angular velocity [rad/s]

% Initialize history arrays
history.x = zeros(1, num_steps+1);
history.y = zeros(1, num_steps+1);
history.theta = zeros(1, num_steps+1);
history.v = zeros(1, num_steps+1);
history.omega = zeros(1, num_steps+1);
history.time = zeros(1, num_steps+1);

% Store initial states
history.x(1) = x;
history.y(1) = y;
history.theta(1) = theta;
history.time(1) = 0;

%% Scenario 1: Simple Motion Sequence

% Main simulation loop
for i = 1:num_steps
    % Current time
    t = (i-1) * dt;
    
    % Determine control inputs based on the current stage
    if i < num_steps/3
        % Stage 1: Circular motion
        tau_r = 0.3;
        tau_l = 0.2;
    elseif i < 2*num_steps/3
        % Stage 2: Straight motion
        tau_r = 0.3;
        tau_l = 0.3;
    else
        % Stage 3: Turn right
        tau_r = 0.1;
        tau_l = 0.3;
    end
    
    % Update dynamics (wheel velocities)
    [omega_r, omega_l] = update_dynamics(omega_r, omega_l, tau_r, tau_l, robot, dt);
    
    % Calculate robot velocities from wheel velocities
    [v, omega] = forward_kinematics(omega_r, omega_l, robot);
    
    % Update robot position and orientation
    [x, y, theta] = update_kinematics(x, y, theta, v, omega, dt);
    
    % Store states in history
    history.x(i+1) = x;
    history.y(i+1) = y;
    history.theta(i+1) = theta;
    history.v(i+1) = v;
    history.omega(i+1) = omega;
    history.time(i+1) = t + dt;
end

% Plot the results
plot_robot_path(history);
animate_robot(history, robot);

%% Scenario 2: Figure-8 Path Following

% Reset robot state
x = 0;
y = 0;
theta = 0;
omega_r = 0;
omega_l = 0;

% Initialize history arrays for scenario 2
history2 = history;
history2.x(1) = x;
history2.y(1) = y;
history2.theta(1) = theta;
history2.time(1) = 0;

% Define figure-8 path
t_values = linspace(0, 2*pi, num_steps+1);
scale_x = 2.0;
scale_y = 1.0;
path_x = scale_x * sin(t_values);
path_y = scale_y * sin(t_values) .* cos(t_values);

% Controller gains
kp_v = 2.0;     % Proportional gain for velocity
kp_omega = 5.0; % Proportional gain for angular velocity

% Main simulation loop for path following
for i = 1:num_steps
    % Current time
    t = (i-1) * dt;
    
    % Current position and orientation
    current_x = x;
    current_y = y;
    current_theta = theta;
    
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
    
    % Calculate control inputs using PID
    v = kp_v * distance;  % Linear velocity proportional to distance
    omega = kp_omega * heading_error;  % Angular velocity proportional to heading error
    
    % Limit the control inputs
    v = max(min(v, 1.0), -1.0);  % Limit to [-1, 1]
    omega = max(min(omega, 2.0), -2.0);  % Limit to [-2, 2]
    
    % Convert to wheel velocities
    [omega_l, omega_r] = inverse_kinematics(v, omega, robot);
    
    % Convert wheel velocities to torques (simplified)
    tau_l = omega_l * 0.1;  % Simple gain
    tau_r = omega_r * 0.1;  % Simple gain
    
    % Update dynamics
    [omega_r, omega_l] = update_dynamics(omega_r, omega_l, tau_r, tau_l, robot, dt);
    
    % Forward kinematics to get actual robot velocities
    [v, omega] = forward_kinematics(omega_r, omega_l, robot);
    
    % Update kinematics
    [x, y, theta] = update_kinematics(x, y, theta, v, omega, dt);
    
    % Store states in history
    history2.x(i+1) = x;
    history2.y(i+1) = y;
    history2.theta(i+1) = theta;
    history2.v(i+1) = v;
    history2.omega(i+1) = omega;
    history2.time(i+1) = t + dt;
end

% Plot the results for figure-8 path following
figure;
plot(path_x, path_y, 'r--', 'LineWidth', 2, 'DisplayName', 'Target Path');
hold on;
plot(history2.x, history2.y, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Robot Path');
plot(history2.x(1), history2.y(1), 'go', 'MarkerSize', 10, 'DisplayName', 'Start');
plot(history2.x(end), history2.y(end), 'mo', 'MarkerSize', 10, 'DisplayName', 'End');
grid on;
axis equal;
xlabel('X [m]');
ylabel('Y [m]');
title('Path Following - Figure-8');
legend('show');

% Plot error over time
calculate_path_error(history2, path_x, path_y);

% Animate the path following
animate_robot(history2, robot, path_x, path_y);

%% Scenario 3: Kinematics vs Dynamics Comparison

% Initialize two robots: one with just kinematics, one with full dynamics
x_kin = 0;
y_kin = 0;
theta_kin = 0;
v_kin = 0;
omega_kin = 0;

x_dyn = 0;
y_dyn = 0;
theta_dyn = 0;
omega_r_dyn = 0;
omega_l_dyn = 0;

% Create a robot with higher mass and inertia to see more pronounced effects
robot_dyn = robot;
robot_dyn.mass = 5.0;
robot_dyn.I = 0.5;
robot_dyn.B_r = 0.1;  % Higher friction
robot_dyn.B_l = 0.1;

% Recalculate dynamic parameters
robot_dyn.J_r = (robot_dyn.mass * robot_dyn.wheel_radius^2 / 4) + ...
               (robot_dyn.I * robot_dyn.wheel_radius^2 / robot_dyn.wheel_base^2);
robot_dyn.J_l = robot_dyn.J_r;
robot_dyn.J_rl = (robot_dyn.mass * robot_dyn.wheel_radius^2 / 4) - ...
                (robot_dyn.I * robot_dyn.wheel_radius^2 / robot_dyn.wheel_base^2);

% Initialize history arrays for comparison
history_kin = history;
history_kin.x(1) = x_kin;
history_kin.y(1) = y_kin;
history_kin.theta(1) = theta_kin;
history_kin.v(1) = v_kin;
history_kin.omega(1) = omega_kin;
history_kin.time(1) = 0;

history_dyn = history;
history_dyn.x(1) = x_dyn;
history_dyn.y(1) = y_dyn;
history_dyn.theta(1) = theta_dyn;
history_dyn.v(1) = 0;
history_dyn.omega(1) = 0;
history_dyn.time(1) = 0;

% Shorter simulation time for comparison
sim_time_comp = 5.0;
num_steps_comp = sim_time_comp/dt;

% Run comparison simulation
for i = 1:num_steps_comp
    % Current time
    t = (i-1) * dt;
    
    % Input: sudden acceleration then sudden stop
    if i < num_steps_comp/3
        % Accelerate
        tau_l = 0.3;
        tau_r = 0.3;
        
        % For kinematic model, directly set velocities (instantaneous)
        v_kin = 0.5;      % Direct velocity control
        omega_kin = 0.0;  % No rotation
    else
        % Sudden stop
        tau_l = 0.0;
        tau_r = 0.0;
        
        % For kinematic model, instantaneous stop
        v_kin = 0.0;
        omega_kin = 0.0;
    end
    
    % Update dynamics model
    [omega_r_dyn, omega_l_dyn] = update_dynamics(omega_r_dyn, omega_l_dyn, tau_r, tau_l, robot_dyn, dt);
    [v_dyn, omega_dyn] = forward_kinematics(omega_r_dyn, omega_l_dyn, robot_dyn);
    [x_dyn, y_dyn, theta_dyn] = update_kinematics(x_dyn, y_dyn, theta_dyn, v_dyn, omega_dyn, dt);
    
    % Update kinematic model
    [x_kin, y_kin, theta_kin] = update_kinematics(x_kin, y_kin, theta_kin, v_kin, omega_kin, dt);
    
    % Store states
    history_kin.x(i+1) = x_kin;
    history_kin.y(i+1) = y_kin;
    history_kin.theta(i+1) = theta_kin;
    history_kin.v(i+1) = v_kin;
    history_kin.omega(i+1) = omega_kin;
    history_kin.time(i+1) = t + dt;
    
    history_dyn.x(i+1) = x_dyn;
    history_dyn.y(i+1) = y_dyn;
    history_dyn.theta(i+1) = theta_dyn;
    history_dyn.v(i+1) = v_dyn;
    history_dyn.omega(i+1) = omega_dyn;
    history_dyn.time(i+1) = t + dt;
end

% Plot comparison
figure;
subplot(1, 2, 1);
plot(history_kin.x, history_kin.y, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Kinematic Model');
hold on;
plot(history_dyn.x, history_dyn.y, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Dynamic Model');
grid on;
axis equal;
xlabel('X [m]');
ylabel('Y [m]');
title('Path Comparison');
legend('show');

subplot(1, 2, 2);
plot(history_kin.time, history_kin.v, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Kinematic Model');
hold on;
plot(history_dyn.time, history_dyn.v, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Dynamic Model');
grid on;
xlabel('Time [s]');
ylabel('Linear Velocity [m/s]');
title('Velocity Comparison');
legend('show');

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

function plot_robot_path(history)
    % Plot robot path and velocities
    figure;
    
    % Plot path
    subplot(1, 2, 1);
    plot(history.x, history.y, 'b-', 'LineWidth', 1.5);
    hold on;
    plot(history.x(1), history.y(1), 'go', 'MarkerSize', 10, 'DisplayName', 'Start');
    plot(history.x(end), history.y(end), 'ro', 'MarkerSize', 10, 'DisplayName', 'End');
    grid on;
    axis equal;
    xlabel('X [m]');
    ylabel('Y [m]');
    title('Robot Path');
    legend('show');
    
    % Plot velocities
    subplot(1, 2, 2);
    plot(history.time, history.v, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Linear Velocity [m/s]');
    hold on;
    plot(history.time, history.omega, 'g-', 'LineWidth', 1.5, 'DisplayName', 'Angular Velocity [rad/s]');
    grid on;
    xlabel('Time [s]');
    ylabel('Velocity');
    title('Robot Velocities');
    legend('show');
end

function calculate_path_error(history, path_x, path_y)
    % Calculate and plot path following error
    % Ensure path and history have the same length
    if length(history.x) ~= length(path_x)
        error('History and path must have the same length');
    end
    
    % Calculate error at each time step
    errors = zeros(size(history.x));
    for i = 1:length(history.x)
        dx = history.x(i) - path_x(i);
        dy = history.y(i) - path_y(i);
        errors(i) = sqrt(dx^2 + dy^2);
    end
    
    % Plot the error over time
    figure;
    plot(history.time, errors, 'r-', 'LineWidth', 1.5);
    grid on;
    xlabel('Time [s]');
    ylabel('Path Following Error [m]');
    title('Path Following Error Over Time');
end

function animate_robot(history, robot, path_x, path_y)
    % Create animation of the robot movement
    
    % Create figure
    fig = figure();
    ax = axes('Parent', fig);
    hold(ax, 'on');
    
    if nargin > 2
        % If path is provided, plot it
        plot(ax, path_x, path_y, 'r--', 'LineWidth', 1.5);
    end
    
    % Set axis limits
    axis_margin = 0.5;
    xlim(ax, [min(history.x)-axis_margin, max(history.x)+axis_margin]);
    ylim(ax, [min(history.y)-axis_margin, max(history.y)+axis_margin]);
    grid(ax, 'on');
    axis(ax, 'equal');
    xlabel(ax, 'X [m]');
    ylabel(ax, 'Y [m]');
    title(ax, 'Mobile Robot Simulation');
    
    % Robot visualization settings
    robot_length = robot.body_length;
    robot_width = robot.body_width;
    wheel_radius = robot.wheel_radius;
    wheel_base = robot.wheel_base;
    
    % Initial position for robot parts
    x_init = history.x(1);
    y_init = history.y(1);
    theta_init = history.theta(1);
    
    % Calculate initial positions
    x_robot = [x_init - (robot_length/2), x_init + (robot_length/2), ...
              x_init + (robot_length/2), x_init - (robot_length/2)];
    y_robot = [y_init - (robot_width/2), y_init - (robot_width/2), ...
              y_init + (robot_width/2), y_init + (robot_width/2)];
    
    % Initialize robot graphics objects
    h_robot = patch(ax, 'XData', x_robot, 'YData', y_robot, 'FaceColor', 'b', 'FaceAlpha', 0.7);
    
    % Initial wheel positions
    wheel_offset = wheel_base / 2;
    left_x = x_init - wheel_offset;
    left_y = y_init;
    right_x = x_init + wheel_offset;
    right_y = y_init;
    
    % Left wheel
    x_left = [left_x - wheel_radius, left_x + wheel_radius, ...
             left_x + wheel_radius, left_x - wheel_radius];
    y_left = [left_y - wheel_radius, left_y - wheel_radius, ...
             left_y + wheel_radius, left_y + wheel_radius];
    h_left_wheel = patch(ax, 'XData', x_left, 'YData', y_left, 'FaceColor', 'k');
    
    % Right wheel
    x_right = [right_x - wheel_radius, right_x + wheel_radius, ...
              right_x + wheel_radius, right_x - wheel_radius];
    y_right = [right_y - wheel_radius, right_y - wheel_radius, ...
              right_y + wheel_radius, right_y + wheel_radius];
    h_right_wheel = patch(ax, 'XData', x_right, 'YData', y_right, 'FaceColor', 'k');
    
    % Direction indicator
    dir_length = robot_length / 2;
    h_direction = plot(ax, [x_init, x_init + dir_length * cos(theta_init)], ...
                     [y_init, y_init + dir_length * sin(theta_init)], ...
                     'r-', 'LineWidth', 2);
    
    % Robot path trace
    h_path = plot(ax, history.x(1), history.y(1), 'b-', 'LineWidth', 1.5);
    
    % Add time display
    h_time = text(ax, min(history.x), max(history.y) + 0.2, 'Time: 0.0s');
    
    % Animation step size (to control speed)
    step = max(1, floor(length(history.x) / 100));
    
    % Animation
    for i = 1:step:length(history.x)
        % Current position and orientation
        x = history.x(i);
        y = history.y(i);
        theta = history.theta(i);
        
        % Update robot body
        x_robot = [x - (robot_length/2) * cos(theta) - (robot_width/2) * sin(theta), ...
                  x + (robot_length/2) * cos(theta) - (robot_width/2) * sin(theta), ...
                  x + (robot_length/2) * cos(theta) + (robot_width/2) * sin(theta), ...
                  x - (robot_length/2) * cos(theta) + (robot_width/2) * sin(theta)];
        
        y_robot = [y - (robot_length/2) * sin(theta) + (robot_width/2) * cos(theta), ...
                  y + (robot_length/2) * sin(theta) + (robot_width/2) * cos(theta), ...
                  y + (robot_length/2) * sin(theta) - (robot_width/2) * cos(theta), ...
                  y - (robot_length/2) * sin(theta) - (robot_width/2) * cos(theta)];
        
        set(h_robot, 'XData', x_robot, 'YData', y_robot);
        
        % Update wheels
        wheel_offset = wheel_base / 2;
        left_x = x - wheel_offset * sin(theta);
        left_y = y + wheel_offset * cos(theta);
        right_x = x + wheel_offset * sin(theta);
        right_y = y - wheel_offset * cos(theta);
        
        % Left wheel
        x_left = [left_x - wheel_radius, left_x + wheel_radius, ...
                 left_x + wheel_radius, left_x - wheel_radius];
        y_left = [left_y - wheel_radius, left_y - wheel_radius, ...
                 left_y + wheel_radius, left_y + wheel_radius];
        set(h_left_wheel, 'XData', x_left, 'YData', y_left);
        
        % Right wheel
        x_right = [right_x - wheel_radius, right_x + wheel_radius, ...
                  right_x + wheel_radius, right_x - wheel_radius];
        y_right = [right_y - wheel_radius, right_y - wheel_radius, ...
                  right_y + wheel_radius, right_y + wheel_radius];
        set(h_right_wheel, 'XData', x_right, 'YData', y_right);
        
        % Direction indicator
        dir_length = robot_length / 2;
        set(h_direction, 'XData', [x, x + dir_length * cos(theta)], ...
                         'YData', [y, y + dir_length * sin(theta)]);
        
        % Update path
        set(h_path, 'XData', history.x(1:i), 'YData', history.y(1:i));
        
        % Update time display
        set(h_time, 'String', sprintf('Time: %.1fs', history.time(i)));
        
        % Pause to control animation speed
        pause(0.01);
        drawnow;
    end
end