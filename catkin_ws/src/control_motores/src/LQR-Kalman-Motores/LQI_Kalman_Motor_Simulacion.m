clc;
clear;
close all;

% ----------------- Parámetros del motor DC 12v -----------------

b = 0.017447;     % Fricción (N·m·s/rad)
J = 0.0056459;    % Inercia (kg·m²)
Kt = 0.35103;     % Constante de torque (N·m/A)
Ke = Kt;          % Constante de FEM (V·s/rad)
Ra = 10.397;      % Resistencia (Ω)
La = 0.015917;    % Inductancia (H)

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
Cd = C;

% ----------------- Aumento del sistema con integrador -----------------

A_aug = [Ad, zeros(2,1);
        -Cd*Ad, 1];
B_aug = [Bd;
        -Cd*Bd];

Q_aug = diag([0.01, 10, 10]);   % penalización para [i; w; e_int]
R_aug = 10;

[P_aug, ~, ~] = dare(A_aug, B_aug, Q_aug, R_aug);
K_aug = R_aug \ (B_aug' * P_aug);  % K_aug = [K_x, K_i]

% ----------------- Filtro de Kalman discreto -----------------

F_K = Ad;
G_K = Bd;
H_K = eye(2);             % Se observan corriente y velocidad
x_hat = [0; 0];           % Estimación inicial
P_K = diag([1, 1]);       % Covarianza inicial

Q_K = diag([10, 0.05]);   % Ruido de proceso
R_K = diag([0.5, 5]);     % Ruido de medición

% ----------------- Variables iniciales -----------------

x = [0; 0];     % Estados reales [corriente; velocidad]
e_int = 0;      % Estado integrador
u = 0;

% ----------------- Almacenamiento para gráficas -----------------

x_plot = [];
xhat_plot = [];
u_plot = [];
t_plot = [];
ref_plot = [];
err_plot = [];

% ----------------- Simulación discreta -----------------

for t = 0:dt:S
    
    wr = 2 + 5*sin(t) + 2*cos(5*t);  % Referencia variable
    
    % Medición simulada (con ruido)
    z = H_K * x + 2*randn(2,1);
    
    % Estimación (corrección)
    K_K = P_K * H_K' / (H_K * P_K * H_K' + R_K);
    x_hat = x_hat + K_K * (z - H_K * x_hat);
    P_K = (eye(2) - K_K * H_K) * P_K;

    % Error y actualización del integrador
    error = wr - x_hat(2);
    e_int = e_int + error * dt;

    % Estado aumentado para control
    x_aug_hat = [x_hat; e_int];

    % Control LQI
    u = -K_aug * x_aug_hat;

    % Evolución real del sistema
    x = F_K * x + G_K * u + 0.1*randn(2,1);

    % Predicción del estimador
    x_hat = F_K * x_hat + G_K * u;
    P_K = F_K * P_K * F_K' + Q_K;

    % Registro
    x_plot = [x_plot, x];
    xhat_plot = [xhat_plot, x_hat];
    u_plot = [u_plot, u];
    t_plot = [t_plot, t];
    ref_plot = [ref_plot, wr];
    err_plot = [err_plot, error];
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
title("Seguimiento de referencia con LQI + Kalman");

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
