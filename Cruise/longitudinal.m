%% LONGITUDINAL CRUCERO – LQR + h EN EL ESTADO
% 2 perturbaciones (alpha0 y u0) + Bode de DEMANDA DE CONTROL usando Bv (viento)
clc; clear; close all;

load("XSWIFT_h=100_v=30.mat")
plotting = false;
%% ===== Bandas de actuadores (referencia en Bode) =====
ema_bw_hz = 10;          % [Hz]
engine_bw = 2;           % [rad/s]
w_ema = 2*pi*ema_bw_hz;  % [rad/s]
w_eng = engine_bw;       % [rad/s]

%% ===== Modelo base =====
% x = [u; alpha; q; theta]
A     = mdl.lng.A;
Bfull = mdl.lng.B;

% Controles (inputs) = [elevator; thrust]
Bc = Bfull(:,1:2);

% Perturbaciones (viento / disturbios): preferir Bv si existe
if isfield(mdl.lng,'Bv')
    Bv = mdl.lng.Bv;
else
    if size(Bfull,2) > 2
        Bv = Bfull(:,3:end);
    else
        error('No encuentro matriz de perturbaciones: ni mdl.lng.Bv ni columnas 3:end en mdl.lng.B.');
    end
end

%% ===== Agregar altura h como estado =====
% x_aug = [u; alpha; q; theta; h]
% h_dot ≈ u_trim*(theta - alpha)
u_trim = 30;  % [m/s] 

A_aug = zeros(5,5);
B_aug = zeros(5,2);

A_aug(1:4,1:4) = A;
B_aug(1:4,:)   = Bc;

A_aug(5,2) = -u_trim;   % -u_trim * alpha
A_aug(5,4) =  u_trim;   % +u_trim * theta

% Augmentar Bv para incluir h: asumimos que el viento NO entra directo en h_dot
nd = size(Bv,2);
if size(Bv,1) == 4
    Bv_aug = [Bv; zeros(1,nd)];
elseif size(Bv,1) == 5
    Bv_aug = Bv;
else
    error('Bv tiene %d filas. Esperaba 4 (modelo base) o 5 (ya aumentado).', size(Bv,1));
end

%% ===== LQR (MANUAL) =====
% Estados: [u alpha q theta h]
Q = diag([ 2, 1, 15, 10, 0.1 ]);
R = diag([ 1, 60 ]);

K   = lqr(A_aug, B_aug, Q, R);
Acl = A_aug - B_aug*K;

disp('K (2x5) ='); disp(K);
disp('Polos lazo cerrado (aug):'); disp(eig(Acl));

% Controlabilidad 
rCont = rank(ctrb(A_aug, B_aug));
fprintf('Rango controlabilidad (A_aug,B_aug): %d / %d\n', rCont, size(A_aug,1));

%% ===== Tiempo =====
tEnd = 120;
dt   = 0.01;
t    = 0:dt:tEnd;

%% ===== Sistemas para initial() =====
sys_original = ss(A,zeros(4,1), eye(4), 0);  % sistema original sin aumentar
sys_ol = ss(A_aug, zeros(5,1), eye(5), 0);   % open-loop
sys_cl = ss(Acl,   zeros(5,1), eye(5), 0);   % closed-loop con u=-Kx

%% ============================================================
% CASO A: perturbación inicial en alpha (10°)
%% ============================================================
alpha0_deg = 10;
x0_a = zeros(5,1);
x0_a(2) = deg2rad(alpha0_deg);

[x_ol_a, ~] = initial(sys_ol, x0_a, t);
[x_cl_a, ~] = initial(sys_cl, x0_a, t);

u_cmd_a  = -(K * x_cl_a.').';     % Nx2 = [delta_e, T]
de_cmd_a = u_cmd_a(:,1);
T_cmd_a  = u_cmd_a(:,2);

if plotting
    pzmap(sys_original); grid on;
plot_case(t, x_ol_a, x_cl_a, de_cmd_a, T_cmd_a, 'Perturbación: \alpha(0)=10°');
end

%% ============================================================
% CASO B: perturbación inicial en velocidad u (5 m/s)
%% ============================================================
u0_ms = 5;
x0_b = zeros(5,1);
x0_b(1) = u0_ms;

[x_ol_b, ~] = initial(sys_ol, x0_b, t);
[x_cl_b, ~] = initial(sys_cl, x0_b, t);

u_cmd_b  = -(K * x_cl_b.').';
de_cmd_b = u_cmd_b(:,1);
T_cmd_b  = u_cmd_b(:,2);

if plotting
plot_case(t, x_ol_b, x_cl_b, de_cmd_b, T_cmd_b, 'Perturbación: u(0)=5 m/s');
end

%% ============================================================
% BODE DE DEMANDA DE CONTROL usando Bv (lo que vos querías)
% xdot = (A-BK)x + Bv_aug*v
% u    = -Kx
% ==> U/V = -K (sI - Acl)^(-1) Bv_aug
%% ============================================================
sys_u_v = ss(Acl, Bv_aug, -K, zeros(2,nd));   % salidas: [delta_e; T], entradas: v1..v_nd

if plotting
plot_bode_control_demand(sys_u_v, w_ema, w_eng);
end
%% ===================== Local functions =====================

function plot_case(t, x_ol, x_cl, de_cmd, T_cmd, case_title)
    % x_aug = [u; alpha; q; theta; h]
    u_ol      = x_ol(:,1);
    a_ol_rad  = x_ol(:,2);
    q_ol_rad  = x_ol(:,3);
    th_ol_rad = x_ol(:,4);
    h_ol      = x_ol(:,5);

    u_cl      = x_cl(:,1);
    a_cl_rad  = x_cl(:,2);
    q_cl_rad  = x_cl(:,3);
    th_cl_rad = x_cl(:,4);
    h_cl      = x_cl(:,5);

    alpha_ol = rad2deg(a_ol_rad);      alpha_cl = rad2deg(a_cl_rad);
    q_ol     = rad2deg(q_ol_rad);      q_cl     = rad2deg(q_cl_rad);      % deg/s
    th_ol    = rad2deg(th_ol_rad);     th_cl    = rad2deg(th_cl_rad);

    figure; plot(t, alpha_ol,'--', t, alpha_cl,'-'); grid on;
    title([case_title ' | \alpha(t)']); xlabel('s'); ylabel('deg'); legend('Open-loop','Closed-loop');

    figure; plot(t, q_ol,'--', t, q_cl,'-'); grid on;
    title([case_title ' | q(t)']); xlabel('s'); ylabel('deg/s'); legend('Open-loop','Closed-loop');

    figure; plot(t, th_ol,'--', t, th_cl,'-'); grid on;
    title([case_title ' | \theta(t)']); xlabel('s'); ylabel('deg'); legend('Open-loop','Closed-loop');

    figure; plot(t, u_ol,'--', t, u_cl,'-'); grid on;
    title([case_title ' | u(t)']); xlabel('s'); ylabel('m/s'); legend('Open-loop','Closed-loop');

    figure; plot(t, h_ol,'--', t, h_cl,'-'); grid on;
    title([case_title ' | h(t) (desviación)']); xlabel('s'); ylabel('m'); legend('Open-loop','Closed-loop');

    figure; plot(t, de_cmd); grid on;
    title([case_title ' | \delta_e(t)']); xlabel('s'); ylabel('input units');

    figure; plot(t, T_cmd); grid on;
    title([case_title ' | T(t)']); xlabel('s'); ylabel('input units');
end

function plot_bode_control_demand(sys_u_v, w_ema, w_eng)
    % Bode magnitud de demanda de control: |delta_e / v_i| y |T / v_i|
    w = logspace(-3, 2.5, 1200);           % rad/s
    G = freqresp(sys_u_v, w);              % 2 x nd x N
    Mag = abs(G);
    nd = size(G,2);

    vNames = {'u_w','w_w','q_w'};

    figure;
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

    % ---- delta_e ----
    nexttile; hold on; grid on; set(gca,'XScale','log');
    title('Demanda de control: |\delta_e / v_i|');
    xlabel('\omega [rad/s]'); ylabel('Magnitud [dB]');
    for i = 1:nd
        mag_i = squeeze(Mag(1,i,:));
        plot(w, 20*log10(mag_i + 1e-12), 'LineWidth', 1.1);
    end
    xline(w_eng, ':', sprintf('\\omega_{eng}=%.3g', w_eng), 'LineWidth', 1.0);
    xline(w_ema, ':', sprintf('\\omega_{ema}=%.3g', w_ema), 'LineWidth', 1.0);
    legend(vNames, 'Location','eastoutside');

    % ---- T ----
    nexttile; hold on; grid on; set(gca,'XScale','log');
    title('Demanda de control: |T / v_i|');
    xlabel('\omega [rad/s]'); ylabel('Magnitud [dB]');
    for i = 1:nd
        mag_i = squeeze(Mag(2,i,:));
        plot(w, 20*log10(mag_i + 1e-12), 'LineWidth', 1.1);
    end
    xline(w_eng, ':', sprintf('\\omega_{eng}=%.3g', w_eng), 'LineWidth', 1.0);
    xline(w_ema, ':', sprintf('\\omega_{ema}=%.3g', w_ema), 'LineWidth', 1.0);
    legend(vNames, 'Location','eastoutside');
end
K_long = K;

% observador
obs_alpha_k = [A(2,:), Bc(2,:)];
obs_alpha_la = 20;

save('long_cruise.mat','K_long','obs_alpha_k','obs_alpha_la');
fprintf('Saved design to long_cruise.mat\n');
