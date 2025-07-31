clear all
close all

% ----------------- Parámetros medidos del motor DC 12v -----------------

B_Izq = 0.0024079912;     % Fricción (N·m·s/rad)
J_Izq = 5.66646261e-5;     % Inercia (kg·m²)
Ka_Izq = 0.2303506651;      % Constante de torque (N·m/A)
Km_Izq = Ka_Izq;           % Constante de FEM (V·s/rad)
R_Izq = 23.4;         % Resistencia (Ω)
L_Izq = 0.0124;    % Inductancia (H)

% ----------------- Parámetros medidos del motor DC 24v -----------------

B_Der = 0.00685316914;     % Fricción (N·m·s/rad)
J_Der = 9.99790139e-4;     % Inercia (kg·m²)
Ka_Der = 0.5366940211;      % Constante de torque (N·m/A)
Km_Der = Ka_Der;           % Constante de FEM (V·s/rad)
R_Der = 7.2;         % Resistencia (Ω)
L_Der = 0.0124;    % Inductancia (H)