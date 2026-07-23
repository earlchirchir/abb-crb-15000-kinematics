% Clear workspace and command window
clear all
close all
clc

% Define initial and final positions for the end effector in Cartesian space
fprintf('Initializing trajectory parameters...\n');
p_init = [0.5; 0.5; 0.35];   % Initial position [x; y; z]
p_final = [0.5; -0.5; 0.35]; % Final position [x; y; z]


% Calculate the total distance the end effector needs to travel
total_distance = norm(p_final - p_init);

% Define motion constraints for the end effector
vmax = 1.0;  % Maximum end effector velocity in m/s
amax = 2.0;  % Maximum end effector acceleration in m/s^2

% Calculate time parameters for bang-coast-bang profile
Ts = vmax / amax;  % Time needed to reach maximum velocity

% Determine if we need a coast phase by checking if the distance is long enough
% to reach maximum velocity
min_distance_for_coast = vmax^2 / amax;  % Minimum distance needed for coast phase

if total_distance > min_distance_for_coast
    % Long movement - will have coast phase
    T_coast = (total_distance - vmax^2/amax) / vmax;  % Duration of coast phase
    T = 2*Ts + T_coast;  % Total movement time
else
    % Short movement - no coast phase
    T = 2 * sqrt(total_distance/amax);  % Total time for pure bang-bang
    Ts = T/2;  % Time of acceleration phase equals deceleration phase
    vmax = amax * Ts;  % Peak velocity will be less than vmax
end

% Generate time points for the trajectory
num_points = 1000;
t = linspace(0, T, num_points);
fprintf('Trajectory time window: %.2f seconds over %d points.\n', T, num_points);
dt = t(2) - t(1);  % Time step for numerical integration

% Initialize arrays for trajectory profiles
sigma = zeros(1, num_points);      % Position along the path
velocity = zeros(1, num_points);   % Velocity magnitude
acceleration = zeros(1, num_points); % Acceleration magnitude
p = zeros(3, num_points);          % End effector positions
pdot = zeros(3, num_points);       % End effector velocities
pddot = zeros(3, num_points);      % End effector accelerations

fprintf('Initial Parameters:\n');
fprintf('  p_init: [%.2f, %.2f, %.2f]\n', p_init(1), p_init(2), p_init(3));
fprintf('  p_final: [%.2f, %.2f, %.2f]\n', p_final(1), p_final(2), p_final(3));
fprintf('  Total Distance: %.4f m\n', total_distance);
fprintf('  vmax: %.2f m/s, amax: %.2f m/s^2\n', vmax, amax);
fprintf('  Calculated Total Time T: %.4f s\n', T);

% Generate the time-optimal bang-coast-bang trajectory (Vectorized)
fprintf('Starting acceleration, coast, and deceleration phases...\n');

% Logical masks for the three phases
accel_mask = (t <= Ts);
coast_mask = (t > Ts) & (t <= (T - Ts));
decel_mask = (t > (T - Ts));

% Acceleration Phase
acceleration(accel_mask) = amax;
velocity(accel_mask) = amax * t(accel_mask);
sigma(accel_mask) = 0.5 * amax * t(accel_mask).^2;

% Coast Phase
acceleration(coast_mask) = 0;
velocity(coast_mask) = vmax;
sigma(coast_mask) = vmax * t(coast_mask) - 0.5 * vmax^2 / amax;

% Deceleration Phase
time_remaining = T - t(decel_mask);
acceleration(decel_mask) = -amax;
velocity(decel_mask) = amax * time_remaining;
sigma(decel_mask) = total_distance - 0.5 * amax * time_remaining.^2;

% Convert scalar motion profiles into Cartesian trajectories
direction = (p_final - p_init) / total_distance;  % Unit vector of motion

% Calculate position, velocity, and acceleration vectors
p = p_init + direction * sigma;
pdot = direction * velocity;
pddot = direction * acceleration;

% Find initial joint configuration using gradient method
fprintf('Initial trajectory calculated. Starting inverse kinematics solver...\n');
q = zeros(6, 1);  % Initial guess for joint angles
alpha = 0.01;     % Learning rate
error_threshold = 1e-6;
max_iter = 10000;
iteration = 1;
error_norm = inf;

% Gradient descent to find initial configuration
fprintf('Starting gradient descent to find initial configuration...\n');
while error_norm > error_threshold && iteration <= max_iter
    % Get current end-effector position
    p_current = dirkin_ABB(q(1), q(2), q(3), q(4), q(5));
    
    % Calculate position error
    error = p_init - p_current;
    error_norm = norm(error);
    
    % Get Jacobian
    J = jacobian_ABB(q(1), q(2), q(3), q(4), q(5));
    
    % Update joint angles
    q = q + alpha * J' * error;
    
    if mod(iteration, 1000) == 0
        fprintf('  Iteration %d: Error Norm = %.6f, q = [%.3f, %.3f, %.3f, ...]\n', ...
            iteration, error_norm, q(1), q(2), q(3));
    end
    
    iteration = iteration + 1;
end
fprintf('Gradient descent finished.\n');
fprintf('Final initial joint configuration q (degrees): [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\n', ...
    rad2deg(q(1)), rad2deg(q(2)), rad2deg(q(3)), rad2deg(q(4)), rad2deg(q(5)), rad2deg(q(6)));

% Initialize arrays for joint trajectories
q_history = zeros(num_points, 6);
qdot_history = zeros(num_points, 6);
q_history(1,:) = q';  % Store initial configuration

% Perform inverse differential kinematics to follow the Cartesian trajectory
fprintf('Starting inverse differential kinematics...\n');
for i = 2:num_points
    % Get current Jacobian
    J = jacobian_ABB(q(1), q(2), q(3), q(4), q(5));
    
    % Monitor kinematics outputs periodically
    if mod(i, 100) == 0
        p_ee = dirkin_ABB(q(1), q(2), q(3), q(4), q(5));
        fprintf('Step %d: Pos=[%.3f, %.3f, %.3f], J-norm=%.4f\n', i, p_ee(1), p_ee(2), p_ee(3), norm(J, 'fro'));
    end
    
    % Optional logging for the first and last steps
    if i == 2 || i == num_points
        fprintf('IK Step %d/%d:\n', i, num_points);
        fprintf('  Jacobian norm: %.4f\n', norm(J, 'fro'));
        fprintf('  Target velocity: [%.2f, %.2f, %.2f]\n', pdot(1,i), pdot(2,i), pdot(3,i));
    end
    
    % Calculate joint velocities using pseudoinverse
    qdot = pinv(J) * pdot(:,i);

    % Store joint velocities
    qdot_history(i,:) = qdot';
    
    % Update joint positions through integration
    q = q + qdot * dt;
    
    % Store joint positions
    q_history(i,:) = q';
    
    % Optional: Verify end-effector position matches desired trajectory
    p_actual = dirkin_ABB(q(1), q(2), q(3), q(4), q(5));
    tracking_error = norm(p(:,i) - p_actual);
    
    % If tracking error becomes too large, we might need to adjust
    if tracking_error > 0.01  % 1cm threshold
        fprintf('Warning: Large tracking error (%.4f m) at time %.2f seconds\n', tracking_error, t(i));
    end
end
fprintf('Inverse differential kinematics finished.\n');

% Plot the results
figure('Name', 'End Effector Trajectory Profiles');
subplot(3,1,1)
plot(t, acceleration, 'LineWidth', 2);
title('End Effector Acceleration Profile');
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
grid on;

fprintf('Plotting trajectory profiles...\n');
subplot(3,1,2)
plot(t, velocity, 'LineWidth', 2);
title('End Effector Velocity Profile');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
grid on;

subplot(3,1,3)
plot(t, sigma, 'LineWidth', 2);
title('End Effector Position Along Path');
xlabel('Time [s]');
ylabel('Distance [m]');
grid on;

% Plot 3D trajectory
figure('Name', 'End Effector Path');
plot3(p(1,:), p(2,:), p(3,:), 'b-', 'LineWidth', 2);
hold on;
plot3(p_init(1), p_init(2), p_init(3), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot3(p_final(1), p_final(2), p_final(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
grid on;
title('End Effector Path');
fprintf('Trajectory accomplishment complete. Figures generated.\n');
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
legend('Path', 'Start', 'End');

% Plot joint angles
figure('Name', 'Joint Trajectories');
plot(t, rad2deg(q_history), 'LineWidth', 2);
title('Joint Angles vs Time');
xlabel('Time [s]');
ylabel('Angle [deg]');
grid on;
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');

q_history_RoboDK = q_history;
q_history_RoboDK(:,2) = q_history_RoboDK(:,2)+pi/2;
% Export joint trajectories to CSV
joint_data = array2table([t', rad2deg(q_history_RoboDK)], 'VariableNames', ...
    {'Time', 'Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'});
writetable(joint_data, 'joint_trajectories.csv');


figure('Name', 'Bang-Coast-Bang Velocity Profile');
plot(t, velocity, 'LineWidth', 2);
hold on;
plot([0, Ts], [vmax, vmax], 'r--');
plot([T-Ts, T], [vmax, vmax], 'r--');
title('Bang-Coast-Bang Velocity Profile');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
text(Ts/2, vmax/2, 'Acceleration', 'HorizontalAlignment', 'center');
text(T-Ts/2, vmax/2, 'Deceleration', 'HorizontalAlignment', 'center');
text(T/2, vmax*1.1, 'Coast', 'HorizontalAlignment', 'center');
text(Ts, 0, 'Ts', 'VerticalAlignment', 'top');
text(T-Ts, 0, 'T-Ts', 'VerticalAlignment', 'top');
text(0, vmax, 'vmax', 'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom');
grid on;



figure('Name', 'Joint Velocities');
plot(t, rad2deg(qdot_history), 'LineWidth', 2);
title('Joint Velocities vs Time');
xlabel('Time [s]');
ylabel('Angular Velocity [deg]');
legend('Joint 1', 'Joint 2', 'Joint 3', 'Joint 4', 'Joint 5', 'Joint 6');
grid on;

% Calculate tracking error
tracking_error = zeros(1, num_points);
for i = 1:num_points
    p_actual = dirkin_ABB(q_history(i,1), q_history(i,2), q_history(i,3), q_history(i,4), q_history(i,5));
    tracking_error(i) = norm(p(:,i) - p_actual);
end

figure('Name', 'Tracking Error');
plot(t, tracking_error, 'LineWidth', 2);
title('End Effector Tracking Error');
xlabel('Time [s]');
ylabel('Error [m]');
yline(0.01, 'r--', 'LineWidth', 1.5);
text(T, 0.01, '1cm threshold', 'VerticalAlignment', 'bottom', 'HorizontalAlignment', 'right', 'Color', 'r');
grid on;


% Plot 2D trajectory
figure('Name', 'End Effector Path');
plot(p(2,:), p(3,:), 'b-', 'LineWidth', 2);
hold on;
plot(p_init(2), p_init(3), 'go', 'MarkerSize', 10, 'LineWidth', 2);
plot(p_final(2), p_final(3), 'ro', 'MarkerSize', 10, 'LineWidth', 2);
grid on;
title('End Effector Path');
xlabel('Y [m]');
ylabel('Z [m]');
legend('Path', 'Start', 'End');
axis equal;


figure('Name', '3D Trajectory Animation');

% Plot the full trajectory
plot3(p(1,:), p(2,:), p(3,:), 'b-', 'LineWidth', 2);
hold on;

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('End Effector Trajectory');
grid on;
axis equal;

% Create a marker for the current position
h_marker = plot3(p(1,1), p(2,1), p(3,1), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

% Create text objects for time, speed, and acceleration
h_time = text(min(p(1,:)), max(p(2,:)), max(p(3,:)), '', 'VerticalAlignment', 'top');
h_speed = text(min(p(1,:)), max(p(2,:)), max(p(3,:))-0.05, '', 'VerticalAlignment', 'top');
h_accel = text(min(p(1,:)), max(p(2,:)), max(p(3,:))-0.1, '', 'VerticalAlignment', 'top');

% Animate the motion
for i = 1:num_points
    % Update marker position
    set(h_marker, 'XData', p(1,i), 'YData', p(2,i), 'ZData', p(3,i));
    
    % Update text information
    set(h_time, 'String', sprintf('Time: %.2f s', t(i)));
    set(h_speed, 'String', sprintf('Speed: %.2f m/s', velocity(i)));
    set(h_accel, 'String', sprintf('Acceleration: %.2f m/s^2', acceleration(i)));
    
    % Pause briefly to create animation effect
    pause(0.01);
end


