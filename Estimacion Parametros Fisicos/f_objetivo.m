function obj = f_objetivo(p,t,u,Corriente_Real, Velocidad_Real)
    X = motor_simulate(p,t,u, Corriente_Real, Velocidad_Real);
    Error_I = (X(:,1) - Corriente_Real) / std(Corriente_Real);
    Error_W  = (X(:,2) - Velocidad_Real) / std(Velocidad_Real);
    obj = 3 * sum(Error_W.^2) + sum(Error_I.^2);
end