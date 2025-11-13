#!/usr/bin/env python3

import numpy as np
import pandas as pd
from scipy.integrate import solve_ivp
from scipy.optimize import differential_evolution
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
import json
import os

def detect_column(df, choices):
    for c in choices:
        for col in df.columns:
            if col.lower() == c.lower():
                return col
    for c in choices:
        for col in df.columns:
            if c.lower() in col.lower():
                return col
    return None

def load_data(csv_path, Vcc=12.0):
    df = pd.read_csv(csv_path)
    tcol = detect_column(df, ['t', 'time', 'tiempo'])
    icol = detect_column(df, ['I', 'CorrienteD_mA', 'current'])
    wcol = detect_column(df, ['omega', 'RPMD', 'rad/s', 'w', 'velocidad', 'rpm'])
    ucol = detect_column(df, ['u', 'u_v', 'voltage', 'voltaje', 'pwm'])

    if tcol is None or icol is None or wcol is None or ucol is None:
        raise ValueError(f"No pude detectar columnas correctamente. Columnas detectadas: {df.columns.tolist()}")

    t = df[tcol].values.astype(float)
    i_meas = df[icol].values.astype(float)
    u_raw = df[ucol].values.astype(float)
    w_meas_raw = df[wcol].values.astype(float)

    if np.mean(w_meas_raw) > 100:
        w_meas = w_meas_raw * 2.0 * np.pi / 60.0
    else:
        w_meas = w_meas_raw.astype(float)

    if 'pwm' in ucol.lower() or (u_raw.min() >= -255 and u_raw.max() <= 255 and np.all(np.mod(u_raw,1)==0)):
        if u_raw.min() < 0:
            u_V = (u_raw / 255.0) * Vcc
        else:
            u_V = (u_raw / 255.0) * Vcc
    else:
        u_V = u_raw

    return t, u_V, i_meas, w_meas, df

def simulate_motor_params(params, t, u_array, method='RK45'):
    Rm, Lm, Jm, Bm, Km = params
    Ac = np.array([[-Rm / Lm, -Km / Lm],
                   [ Km / Jm, -Bm / Jm]])
    Bc = np.array([[1.0 / Lm],
                   [0.0]])
    u_func = interp1d(t, u_array, kind='previous', bounds_error=False, fill_value=(u_array[0], u_array[-1]))
    def rhs(ti, x):
        ui = float(u_func(ti))
        return (Ac @ x.reshape(-1,1) + Bc * ui).flatten()

    x0 = np.zeros(2)
    sol = solve_ivp(fun=rhs, t_span=(t[0], t[-1]), y0=x0, t_eval=t, method=method, rtol=1e-6, atol=1e-9)
    x = sol.y.T
    i_sim = x[:,0]
    w_sim = x[:,1]
    return i_sim, w_sim

def cost_function(params, t, u_V, i_meas, w_meas, weights=(1.0, 1.0), method='RK45'):
    if np.any(np.array(params) <= 0):
        return 1e6
    try:
        i_sim, w_sim = simulate_motor_params(params, t, u_V, method=method)
    except Exception as e:
        print("Integracion fallo para params:", params, "->", e)
        return 1e6

    rmse_i = np.sqrt(np.mean((i_sim - i_meas)**2))
    rmse_w = np.sqrt(np.mean((w_sim - w_meas)**2))

    norm_i = np.sqrt(np.mean(i_meas**2)) + 1e-9
    norm_w = np.sqrt(np.mean(w_meas**2)) + 1e-9

    score = weights[0] * (rmse_i / norm_i) + weights[1] * (rmse_w / norm_w)
    return float(score)

def identify_from_csv(csv_path,
                      Vcc=12.0,
                      popsize=15,
                      maxiter=60,
                      workers=1,
                      method='RK45',
                      bounds=None,
                      weights=(1.0, 1.0),
                      save_result="de_result.json"):
    t, u_V, i_meas, w_meas, df = load_data(csv_path, Vcc=Vcc)
    if bounds is None:
        bounds = [
            (1e-2, 50.0),    # Rm [Ohm]
            (1e-6, 1.0),     # Lm [H]
            (1e-8, 1e-1),    # Jm [kg m^2]
            (1e-6, 1.0),     # Bm [N m s]
            (1e-4, 5.0)      # Km [N m/A or V s / rad]
        ]
    def wrapped_cost(x):
        return cost_function(x, t, u_V, i_meas, w_meas, weights=weights, method=method)

    best_so_far = {"x": None, "fun": np.inf}
    def callback(xk, convergence=0):
        nonlocal best_so_far
        val = wrapped_cost(xk)
        if val < best_so_far["fun"]:
            best_so_far["fun"] = val
            best_so_far["x"] = xk.copy()
            print(f"[MEJOR] fun={val:.6e} params={xk}")
        return False

    print("Iniciando Differential Evolution...")
    result = differential_evolution(wrapped_cost, bounds=bounds, strategy='best1bin',
                                    maxiter=maxiter, popsize=popsize, tol=1e-6,
                                    mutation=(0.5, 1), recombination=0.7,
                                    workers=workers, callback=callback, polish=True, seed=1234)

    print("DE terminado. Mejor fun:", result.fun)
    print("Mejores parametros:", result.x)

    i_sim, w_sim = simulate_motor_params(result.x, t, u_V, method=method)
    out_df = pd.DataFrame({
        "t_s": t,
        "u_V": u_V,
        "i_meas_A": i_meas,
        "i_sim_A": i_sim,
        "omega_meas_rad_s": w_meas,
        "omega_sim_rad_s": w_sim
    })
    out_csv = os.path.splitext(csv_path)[0] + "_fit_compare.csv"
    out_df.to_csv(out_csv, index=False)
    print("Comparativa guardada en:", out_csv)

    res_info = {
        "x": result.x.tolist(),
        "fun": float(result.fun),
        "message": result.message,
        "nfev": int(result.nfev),
        "nit": int(result.nit)
    }
    with open(save_result, 'w') as f:
        json.dump(res_info, f, indent=2)

    return result, out_csv

if __name__ == "__main__":
    csv_path = "mediciones_motores.csv"   
    result, compare_csv = identify_from_csv(csv_path,
                                            Vcc=12.0,
                                            popsize=12,
                                            maxiter=80,
                                            workers=1,
                                            method='RK45',
                                            weights=(0.6, 0.4),
                                            save_result="de_result.json")
    print("Resultado final:", result.x, "fun:", result.fun)
    comp = pd.read_csv(compare_csv)
    plt.figure(figsize=(8,4))
    plt.plot(comp["t_s"], comp["i_meas_A"], '.', markersize=4, label="i_meas")
    plt.plot(comp["t_s"], comp["i_sim_A"], '-', label="i_sim")
    plt.legend(); plt.xlabel("t [s]"); plt.ylabel("I [A]"); plt.grid(True)

    plt.figure(figsize=(8,4))
    plt.plot(comp["t_s"], comp["omega_meas_rad_s"], '.', markersize=4, label="w_meas")
    plt.plot(comp["t_s"], comp["omega_sim_rad_s"], '-', label="w_sim")
    plt.legend(); plt.xlabel("t [s]"); plt.ylabel("omega [rad/s]"); plt.grid(True)
    plt.show()