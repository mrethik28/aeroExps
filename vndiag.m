W = 1111 * 9.81; %weight of an cessna 172 in newtons
S = 16.2; % wing area
rho = 1.225; %sea-level air density
CL_max_pos = 1.6;
CL_max_neg = -0.8;
n_max = 3.8; % Positive structural limit (load factor)
n_min = -1.52; % Negative structural limit (load factor)
V_NE = 90.3; %never eceed velocity


% --- Calculate Corner Points ---
% Stall speed at n=1
V_S = sqrt((2 * W) / (rho * S * CL_max_pos));

% Maneuvering speed (VA) - the speed where the stall line intersects n_max
V_A = V_S * sqrt(n_max);



V = linspace(0, V_NE, 200);

% Structural Limits
figure('Name', 'V-n Diagram');
hold on;
plot([0 V_NE], [n_max n_max], 'r--', 'LineWidth', 1.5);
plot([0 V_NE], [n_min n_min], 'r--', 'LineWidth', 1.5); 
plot([V_NE V_NE], [n_min n_max], 'r--', 'LineWidth', 1.5, 'HandleVisibility', 'off');

n_pos_stall = (0.5 * rho * V.^2 * S * CL_max_pos) / W;
n_neg_stall = (0.5 * rho * V.^2 * S * CL_max_neg) / W;

V_pos_stall_plot = V(V <= V_A);
n_pos_stall_plot = n_pos_stall(V <= V_A);
plot(V_pos_stall_plot, n_pos_stall_plot, 'b-', 'LineWidth', 2);

V_neg_corner = sqrt((2 * W * n_min) / (rho * S * CL_max_neg));
V_neg_stall_plot = V(V <= V_neg_corner);
n_neg_stall_plot = n_neg_stall(V <= V_neg_corner);
plot(V_neg_stall_plot, n_neg_stall_plot, 'b-', 'LineWidth', 2, 'HandleVisibility', 'off');


% safe flight area
fill([V_S V_A V_NE V_NE V_neg_corner 0 V_S], ...
     [1 n_max n_max n_min n_min 0 1], 'g', 'FaceAlpha', 0.1, 'EdgeColor', 'none');

title('V-n Diagram for a Light Aircraft');
xlabel('True Airspeed (V) [m/s]');
ylabel('Load Factor (n)');
grid on;
axis([0 100 -3 5]);
legend('Structural Limits', 'Aerodynamic (Stall) Limits', 'Safe Flight Envelope', 'Location', 'southeast');
hold off;

    

