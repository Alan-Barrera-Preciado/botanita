clear all
close all

% ----------------- Parámetros medidos del motor DC 12v -----------------

B = 0.0024079912;     % Fricción (N·m·s/rad)
J = 5.66646261e-5;     % Inercia (kg·m²)
Ka = 0.2303506651;      % Constante de torque (N·m/A)
Km = Ka;           % Constante de FEM (V·s/rad)
R = 23.4;         % Resistencia (Ω)
L = 0.0124;    % Inductancia (H)

% ----------------- Parámetros medidos del motor DC 24v -----------------

% B = 0.00685316914;     % Fricción (N·m·s/rad)
% J = 9.99790139e-4;     % Inercia (kg·m²)
% Ka = 0.5366940211;      % Constante de torque (N·m/A)
% Km = Ka;           % Constante de FEM (V·s/rad)
% R = 7.2;         % Resistencia (Ω)
% L = 0.0124;    % Inductancia (H)