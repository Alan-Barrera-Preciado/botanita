function dx = DC_Motor(t,x,u,p0)
    % Modelo de  Motor DC
    % by: Sergio Andres Castaño Giraldo
    % https://controlautomaticoeducacion.com/
    % youtube.com/c/SergioACastañoGiraldo
    %Estados
    i = x(1); %Corriente
    w = x(2); %Velocidad Angular
    
    %Parametros
    J  = p0(1);
    B  = p0(2);
    Ka = p0(3);
    Km = p0(4);
    R  = p0(5);
    L  = p0(6);
    
    Ea = Ka*w;
    Tm = Km*i;
    
    %ecuaciones diferenciales
    di = 1/L*(u - R*i - Ea);
    dw = 1/J*(Tm - B*w);
    dx=[di;dw];