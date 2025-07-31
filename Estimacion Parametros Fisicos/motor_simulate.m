function X = motor_simulate(p,data, Corriente_Real, Velocidad_Real)
        
    t = data(:,1);
    u = data(:,4);
    
    %Estados
    X = zeros(length(t),2);
    X(1,:) = 0; %Corriente
    X(1,2) = 0; %Velocidad
    x0 = [Corriente_Real(1), Velocidad_Real(1)];
    for i = 1:length(t)-1
        tspan = [t(i),t(i+1)];
        %Solución de la EDO
        [~,y] = ode15s(@(t,x)DC_Motor(t,x,u(i),p),tspan,x0);
        x0 = [y(end,1), y(end,2)];
        X(i+1,1) = x0(1);
        X(i+1,2) = x0(2);
    end
        
end