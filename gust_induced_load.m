clear; clc; close all;

%% 1. Define Aircraft and Environment Parameters
params.mass = 1111 * 9.81;
params.wing_area = 16.2;
params.rho = 1.225;
params.g = 9.81;
params.V = 100;

% CL_alpha is the lift curve slope (change in CL per radian of AoA)
params.CL_alpha = 2 * pi;
params.Weight = params.mass * params.g;

% Calculate the Lift Coefficient required for steady, level flight (Lift = Weight)
params.CL_trim = params.Weight / (0.5 * params.rho * params.V^2 * params.wing_area);

%% 2. Define Gust Parameters
params.gust_start_time = 4.0;
params.gust_end_time = 7.0;
params.gust_duration = params.gust_end_time - params.gust_start_time;
params.Wm = 7.0;

%% 3. Set Up and Run the ODE Solver
tspan = [0 10];
initial_altitude = 1000;

% Initial state vector: [x_pos; y_pos; x_vel; y_vel]
initial_state = [0; initial_altitude; params.V; 0];
[t, sol] = ode45(@(t,y) ode_function(t, y, params), tspan, initial_state);

%% 4. Post-Process Results to Calculate Load Factor
% The solver returns the aircraft's motion (state); this loop recalculates
% the forces at each time step to derive the load factor.
num_steps = length(t);
Lift = zeros(num_steps, 1);
load_factor = zeros(num_steps, 1);

for i = 1:num_steps
    time_step = t(i);
    state_step = sol(i, :)';
    
    if time_step >= params.gust_start_time && time_step <= params.gust_end_time
        t_gust = time_step - params.gust_start_time;
        Wg = (params.Wm / 2) * (1 - cos(2 * pi * t_gust / params.gust_duration));
    else
        Wg = 0;
    end
    
    delta_alpha = atan(Wg / params.V);
    CL_total = params.CL_trim + params.CL_alpha * delta_alpha;
    Lift(i) = 0.5 * params.rho * params.V^2 * params.wing_area * CL_total;
    
    % Load factor is the ratio of current lift to the aircraft's weight
    load_factor(i) = Lift(i) / params.Weight;
end

%% 5. Plot the Results
figure('Name', 'Gust Load Analysis', 'NumberTitle', 'off');

subplot(2, 1, 1);
plot(t, load_factor, 'b-', 'LineWidth', 2);
hold on;
yline(1.0, 'r--', 'LineWidth', 1.5, 'Label', 'Steady Flight (n=1)');
title('Load Factor during Gust Encounter');
xlabel('Time (s)');
ylabel('Load Factor (n)');
grid on;
peak_n = max(load_factor);
text(t(end), peak_n, sprintf('Peak n = %.2f', peak_n), 'VerticalAlignment','top', 'HorizontalAlignment','right');

subplot(2, 1, 2);
altitude = sol(:, 2);
plot(t, altitude, 'k-', 'LineWidth', 2);
title('Aircraft Altitude Response');
xlabel('Time (s)');
ylabel('Altitude (m)');
grid on;

%% The ODE Function
function dydt = ode_function(t, y, params)
    % y(1)=x_pos, y(2)=y_pos, y(3)=x_vel, y(4)=y_vel
    vx = y(3);
    vy = y(4);

    if t >= params.gust_start_time && t <= params.gust_end_time
        t_gust = t - params.gust_start_time;
        Wg = (params.Wm / 2) * (1 - cos(2 * pi * t_gust / params.gust_duration));
    else
        Wg = 0;
    end
    
    delta_alpha = atan(Wg / params.V);
    CL_total = params.CL_trim + params.CL_alpha * delta_alpha;
    L = 0.5 * params.rho * params.V^2 * params.wing_area * CL_total;
    
    % Assuming Thrust = Drag to maintain constant forward speed for simplicity
    F_net_x = 0; 
    F_net_y = L - params.Weight;
    
    ax = F_net_x / params.mass;
    ay = F_net_y / params.mass;

    % Return the derivative of the state vector: [vx; vy; ax; ay]
    dydt = [vx; vy; ax; ay];
end

