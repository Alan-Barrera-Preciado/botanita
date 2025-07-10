function obj = f_objetivo(p,data,data2)
    y_real = table2array(data2(:,2));
    X = motor_simulate(p,data);
    % calculate objective
    obj =  sum((X(:,2)-y_real).^2);         
end