close all
clc

data2 = readtable("df_12v.csv");
data3 = readtable("df_24v.csv");
t = data(:,1);
t2 = table2array(data2(:,1));
t3 = table2array(data3(:,1));

Corriente_Sim_Izq = data(:,2);
Corriente_Real_Izq = table2array(data2(:,3));
Velocidad_Sim_Izq = data(:,3);
Velocidad_Real_Izq = table2array(data2(:,2));
u_Izq = data(:,4);

Corriente_Sim_Der = data(:,5);
Corriente_Real_Der = table2array(data3(:,3));
Velocidad_Sim_Der = data(:,6);
Velocidad_Real_Der = table2array(data3(:,2));
u_Der = data(:,7);

%Variables de decisión
p0_Izq = [J_Izq, B_Izq, Ka_Izq, Km_Izq, R_Izq, L_Izq];
p0_Der = [J_Der, B_Der, Ka_Der, Km_Der, R_Der, L_Der];

% Muestra el objetivo inicial
disp(['SSE Objetivo Inicial Izquierdo: ' num2str(f_objetivo(p0_Izq,t,u_Izq,Corriente_Real_Izq, Velocidad_Real_Izq))])
disp(['SSE Objetivo Inicial Derecho: ' num2str(f_objetivo(p0_Der,t,u_Der,Corriente_Real_Der, Velocidad_Real_Der))])


%% Parámetros de Optimización
A = [];
b = [];
Aeq = [];
beq = [];
nlcon = [];

% Restricciones

lb = [   1e-5,    1e-4,  0.02,  0.02,  1,  0.0001]; % lower bound
ub = [   1e-3,    5e-2,  0.60,  0.60,  12, 0.05]; % upper bound
%    [   J,    B, Ka, Km,  R,     L]

% options = optimoptions(@fmincon,'Algorithm','interior-point');
p_Izq = fmincon(@(p_Izq)f_objetivo(p_Izq,t,u_Izq,Corriente_Real_Izq, Velocidad_Real_Izq),p0_Izq,A,b,Aeq,beq,lb,ub,nlcon); %,options);
p_Der = fmincon(@(p_Der)f_objetivo(p_Der,t,u_Der,Corriente_Real_Der, Velocidad_Real_Der),p0_Der,A,b,Aeq,beq,lb,ub,nlcon);

% show final objective
disp(['SSE Objetivo Final Izquierdo: ' num2str(f_objetivo(p_Izq,t,u_Izq,Corriente_Real_Izq, Velocidad_Real_Izq))])
disp(['SSE Objetivo Final Derecho: ' num2str(f_objetivo(p_Der,t,u_Der,Corriente_Real_Der, Velocidad_Real_Der))])

% Parametros del Motor Optimizados
Jo_Izq  = p_Izq(1);
Bo_Izq  = p_Izq(2);
Kao_Izq = p_Izq(3);
Kmo_Izq = p_Izq(4);
Ro_Izq  = p_Izq(5);
Lo_Izq  = p_Izq(6);
disp(['J_Izq: ' num2str(Jo_Izq)])
disp(['B_Izq: ' num2str(Bo_Izq)])
disp(['Ka_Izq: ' num2str(Kao_Izq)])
disp(['Km_Izq: ' num2str(Kmo_Izq)])
disp(['R_Izq: ' num2str(Ro_Izq)])
disp(['L_Izq: ' num2str(Lo_Izq)])

Jo_Der  = p_Der(1);
Bo_Der  = p_Der(2);
Kao_Der = p_Der(3);
Kmo_Der = p_Der(4);
Ro_Der  = p_Der(5);
Lo_Der  = p_Der(6);
disp(['J_Der: ' num2str(Jo_Der)])
disp(['B_Der: ' num2str(Bo_Der)])
disp(['Ka_Der: ' num2str(Kao_Der)])
disp(['Km_Der: ' num2str(Kmo_Der)])
disp(['R_Der: ' num2str(Ro_Der)])
disp(['L_Der: ' num2str(Lo_Der)])

save('ParametrosEstimadosMotorIzquierdo.mat', 'p_Izq');
save('ParametrosEstimadosMotorDerecho.mat', 'p_Der');

%Calcula el la dinámica del modelo con los nuevos parametros
X_Izq = motor_simulate(p_Izq,t,u_Izq, Corriente_Real_Izq, Velocidad_Real_Izq); %Modelo Nuevo
X_Der = motor_simulate(p_Der,t,u_Der, Corriente_Real_Der, Velocidad_Real_Der);

%%

figure(1)
hold on
grid on
title('Motor Izquierdo - Velocidades')
plot(t,Velocidad_Sim_Izq,'LineWidth',2, 'LineStyle', ':')
plot(t,Velocidad_Real_Izq(:,1), 'LineWidth',2, 'LineStyle', '--')
plot(t,X_Izq(:,2), 'LineWidth', 2, 'LineStyle', '-')
ylabel('Velocidad (rad/s)')
legend('Modelo inicial','Motor real', 'Modelo Optimizado')

figure(2)
hold on
grid on
title('Motor Izquierdo - Corriente')
plot(t,Corriente_Sim_Izq,'LineWidth',2, 'LineStyle', ':')
plot(t,Corriente_Real_Izq(:,1), 'LineWidth',2, 'LineStyle', '--')
plot(t,X_Izq(:,1), 'LineWidth', 2, 'LineStyle', '-')
ylabel('Corriente (mA)')
legend('Modelo inicial','Motor real', 'Modelo Optimizado')

figure(3)
hold on
grid on
title('Motor Derecho - Velocidades')
plot(t,Velocidad_Sim_Der,'LineWidth',2, 'LineStyle', ':')
plot(t,Velocidad_Real_Der(:,1), 'LineWidth',2, 'LineStyle', '--')
plot(t,X_Der(:,2), 'LineWidth', 2, 'LineStyle', '-')
ylabel('Velocidad (rad/s)')
legend('Modelo inicial','Motor real', 'Modelo Optimizado')

figure(4)
hold on
grid on
title('Motor Derecho - Corriente')
plot(t,Corriente_Sim_Der,'LineWidth',2, 'LineStyle', ':')
plot(t,Corriente_Real_Der(:,1), 'LineWidth',2, 'LineStyle', '--')
plot(t,X_Der(:,1), 'LineWidth', 2, 'LineStyle', '-')
ylabel('Corriente (mA)')
legend('Modelo inicial','Motor real', 'Modelo Optimizado')