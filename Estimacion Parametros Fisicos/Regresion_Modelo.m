%% Ajuste de parámetros de Motor DC
% By: Sergio Andrés Castaño Giraldo
% https://controlautomaticoeducacion.com/analisis-de-sistemas/modelo-de-motor-dc/


% clc
% clear
% close

data2 = readtable("df_example_24v.csv");
t = data(:,1);
t2 = table2array(data2(:,1));
y_fen = data(:,2);
y_real = table2array(data2(:,2));
u = data(:,4);

%Variables de decisión
p0 = [J, B, Ka, Km, R, L];

% Muestra el objetivo inicial
disp(['Initial SSE Objective: ' num2str(f_objetivo(p0,data,data2))])


%% Parámetros de Optimización
A = [];
b = [];
Aeq = [];
beq = [];
nlcon = [];

% Restricciones
lb = p0*0.8; lb(end)=1e-6;
ub = p0*1.2; ub(end) = 1;
% lb = [   0,    0,  0,  0, 2.2,  1e-6]; % lower bound
% ub = [0.01,0.001,0.9,0.9,  10,  1]; % upper bound
%    [   J,    B, Ka, Km,  R,     L]
% lb = [   0,    0,  0,  0, 0,  0]; % lower bound
% ub = [];
% options = optimoptions(@fmincon,'Algorithm','interior-point');
p = fmincon(@(p)f_objetivo(p,data,data2),p0,A,b,Aeq,beq,lb,ub,nlcon); %,options);

% show final objective
disp(['Final SSE Objective: ' num2str(f_objetivo(p,data,data2))])

% Parametros del Motor Optimizados
Jo  = p(1);
Bo  = p(2);
Kao = p(3);
Kmo = p(4);
Ro  = p(5);
Lo  = p(6);
disp(['J: ' num2str(Jo)])
disp(['B: ' num2str(Bo)])
disp(['Ka=Km: ' num2str(Kao)])
disp(['R: ' num2str(Ro)])
disp(['L: ' num2str(Lo)])

%Calcula el la dinámica del modelo con los nuevos parametros
X = motor_simulate(p,data); %Modelo Nuevo

figure(2)
hold on
grid on
plot(t,y_fen,'LineWidth',2, 'LineStyle', ':')
plot(t,y_real(:,1), 'LineWidth',2, 'LineStyle', '--')
plot(t,X(:,2), 'LineWidth', 2, 'LineStyle', '-')
ylabel('Velocidad (rad/s)')
legend('Modelo inicial','Motor real', 'Modelo Optimizado')

%Voltaje Leido en los Terminales del motor
% Vm = 3.57;


%% Función de transferencias
den = [Lo*Jo Ro*Jo+Lo*Bo Ro*Bo+Kmo*Kao];

%Velocidad angular
Gwvo = tf(Kmo,den);

%Corriente
Givo = tf([Jo Bo],den);

%Posición
Gwto = tf(Kmo,conv(den,[1 0]));

