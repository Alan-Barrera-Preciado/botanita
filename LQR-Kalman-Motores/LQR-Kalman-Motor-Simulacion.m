clc;
clear;
close all;

% ----------------- Parámetros del motor DC 12v -----------------

% b = 0.003353;     % Fricción (N·m·s/rad)
% J = 8.1574e-5;     % Inercia (kg·m²)
% Kt = 0.30765;      % Constante de torque (N·m/A)
% Ke = Kt;           % Constante de FEM (V·s/rad)
% Ra = 33.6954;         % Resistencia (Ω)
% La = 0.42054;    % Inductancia (H)

% ----------------- Parámetros del motor DC 24v -----------------

b = 0.0082235;     % Fricción (N·m·s/rad)
J = 0.0011997;     % Inercia (kg·m²)
Kt = 0.63643;      % Constante de torque (N·m/A)
Ke = Kt;           % Constante de FEM (V·s/rad)
Ra = 8.64;         % Resistencia (Ω)
La = 0.0043199;    % Inductancia (H)


% ----------------- Modelo continuo -----------------

A = [-Ra/La, -Ke/La;
      Kt/J, -b/J];
B = [1/La; 0];
C = [0, 1];  % salida: velocidad
D = 0;

% ----------------- Discretización -----------------

dt = 0.05;               % Paso de tiempo
S = 10;                  % Tiempo total de simulación
sys_c = ss(A, B, C, D);
sys_d = c2d(sys_c, dt, 'zoh');

Ad = sys_d.A;
Bd = sys_d.B;

% ----------------- LQR discreto -----------------

Q = [0.0001 0; 0 5];       % Penalización a estados
R = 50;                  % Penalización al control
[Pd, ~, ~] = dare(Ad, Bd, Q, R);
K = R \ (Bd' * Pd);       % Ganancia óptima discreta
Kr = inv(C * inv(eye(2) - Ad + Bd*K) * Bd);  % Ganancia de referencia

% ----------------- Filtro de Kalman discreto -----------------

F_K = Ad;
G_K = Bd;
H_K = eye(2);             % Se observan corriente y velocidad
x_hat = [0; 0];           % Estimación inicial
P_K = diag([0.05, 0.75]);             % Covarianza inicial

Q_K = diag([0, 0]);  % Ruido de proceso
R_K = diag([0.8, 0.2]); % Ruido de medición

% ----------------- Variables iniciales -----------------

x = [0; 0];     % Estados reales [corriente; velocidad]
u = 0;          % Voltaje aplicad        % Referencia en rad/s

% ----------------- Almacenamiento para gráficas -----------------

x_plot = [];
xhat_plot = [];
u_plot = [];
t_plot = [];
ref_plot = [];
err_plot = [];

% ----------------- Simulación discreta -----------------

for t = 0:dt:S

    wr = 2 + 5*sin(t)+2*cos(t*5);
    % Medición simulada (con ruido)
    z = H_K * x + 0.5*randn(2,1);

    % Estimación (corrección)
    K_K = P_K * H_K' / (H_K * P_K * H_K' + R_K);
    x_hat = x_hat + K_K * (z - H_K * x_hat);
    P_K = (eye(2) - K_K * H_K) * P_K;

    % Control
    u = -K * x_hat + Kr * wr;
    u = min(12, max(-12, u));  % Saturación

    % Evolución real del sistema
    x = F_K * x + G_K * u + 0.05* randn(2,1);

    % Predicción del estimador
    x_hat = F_K * x_hat + G_K * u;
    P_K = F_K * P_K * F_K' + Q_K;

    % Registro
    x_plot = [x_plot, x];
    xhat_plot = [xhat_plot, x_hat];
    u_plot = [u_plot, u];
    t_plot = [t_plot, t];
    ref_plot = [ref_plot, wr];
    err_plot = [err_plot, wr - x_hat(2)];
end

% ----------------- Gráficas -----------------

figure;
hold on; grid on;
plot(t_plot, x_plot(2,:), 'LineWidth', 2);          % Velocidad real
plot(t_plot, xhat_plot(2,:), 'LineWidth', 2);       % Estimación velocidad
plot(t_plot, ref_plot, 'k--', 'LineWidth', 2);      % Referencia
legend("Velocidad real", "Estimación", "Referencia");
xlabel("Tiempo (s)");
ylabel("Velocidad (rad/s)");
title("Seguimiento de referencia con LQR + Kalman");

figure;
plot(t_plot, u_plot, 'LineWidth', 2);
grid on;
xlabel("Tiempo (s)");
ylabel("Voltaje (V)");
title("Voltaje aplicado");

figure;
plot(t_plot, err_plot, 'LineWidth', 2);
grid on;
xlabel("Tiempo (s)");
ylabel("Error (rad/s)");
title("Error de seguimiento");
