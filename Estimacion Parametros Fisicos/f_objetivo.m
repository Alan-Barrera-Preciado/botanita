function obj = f_objetivo(p,data,Corriente_Real, Velocidad_Real)
    X = motor_simulate(p,data, Corriente_Real, Velocidad_Real);
    err_corr = (X(:,1) - Corriente_Real) / std(Corriente_Real);
    err_vel  = (X(:,2) - Velocidad_Real) / std(Velocidad_Real);
    obj = sum(err_vel.^2);
end