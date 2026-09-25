function [de, da, dr, Tcmd] = implementacion_digital_controles( ...
    beta, p, r, phi, psi, psi_ref, ...        % --- lateral
    V_real,V_des, alpha_meas, q, theta, h, h_ref)   % --- longitudinal

% ============================================================
% Salidas (todas normalizadas): da, dr, de, Tcmd  in [-1,1]
% Entradas: estados necesarios
% Cross-track: SOLO opción A -> y se integra con y_dot = V*(psi_e + beta)
% ============================================================

%% ===== Parámetros discretos =====
fs = 1000; %Hz
Ts = 1/fs; %s

%% ===== Memorias internas (necesarias por integración/observador) =====
persistent y_ct alpha_hat u_long_prev
if isempty(y_ct)
    y_ct = 0;                  % cross-track inicial [m]
end
if isempty(alpha_hat)
    alpha_hat = alpha_meas;    % inicialización: arranca en la medición
end
if isempty(u_long_prev)
    u_long_prev = [0; 0];      % [de; T] aplicado en el paso anterior
end

%% ============================================================
% 1) CONTROL LATERAL 
%% ============================================================

% --- Error de rumbo ---
psi_e = wrapToPi_local(psi - psi_ref);

% --- Cross-track (SOLO opción A): integrar y_dot = V*(psi_e + beta) ---
y_dot = V_real * (psi_e + beta);
y_ct  = y_ct + Ts * y_dot;

% --- Vector de estado lateral ---
x_lat = [beta; p; r; phi; psi_e; y_ct];

% --- Ganancia lateral (tu K_lat) ---
K_lat = [ 3.3086    1.0079   -0.3423    2.1300    3.2031    0.0303;
         -6.3827    0.4817   10.6357    1.9398   14.3002    0.1114];

% --- Control lateral ---
u_lat = -K_lat * x_lat;     % [da; dr]
da = sat(u_lat(1), 1.0);
dr = sat(u_lat(2), 1.0);

%% ============================================================
% 2) CONTROL LONGITUDINAL (LQR long + observador de alpha)
%% ============================================================

% ---- Observador de alpha  ----
% obs_alpha_k = [A(2,:), Bc(2,:)]  (1x6)  y obs_alpha_la > 0
obs_alpha_k  = [-0.0287   -7.0486    0.9931    0.0109   -0.1154   -0.4252]; % ejemplo
obs_alpha_la = 20; 

% Modelo para el predictor: alpha_dot_hat = obs_alpha_k*[u; alpha_hat; q; theta; de_prev; T_prev] + la*(alpha_meas-alpha_hat)
u = V_real-V_des;
alpha_dot_model = obs_alpha_k * [u; alpha_hat; q; theta; u_long_prev(1); u_long_prev(2)];
alpha_hat = alpha_hat + Ts * ( alpha_dot_model + obs_alpha_la * (alpha_meas - alpha_hat) );

% --- Estado longitudinal aumentado ---
deltah = h - h_ref;
x_long = [u; alpha_hat; q; theta; deltah];

% --- Ganancia longitudinal (K_long) ---
K_long = [-0.0045    2.3978   -3.8715  -10.7032   -0.3161;
          0.1822    0.0065    0.0023   -0.0375    0.0010];


u_long = -K_long * x_long;     % [de; T]
de   = sat(u_long(1), 1.0);
Tcmd = min(max(u_long(2), 0), 1);

% Guardar inputs aplicados para el próximo paso del observador
u_long_prev = [de; Tcmd];

end

%% ===================== helpers =====================
function x = sat(x, lim)
x = min(max(x, -lim), lim);
end

function ang = wrapToPi_local(ang)
ang = mod(ang + pi, 2*pi) - pi;
end
