clc; clear; close all;

load("XSWIFT_h=100_v=30.mat")
plotting = true;
A  = mdl.lat.A;
B  = mdl.lat.B;

Bu = B(:,1:2);        % actuadores: [ail, rdr]
Bv = B(:,3:5);        % disturbios (según tu modelo): [vw pw rw]

% ===================== Parámetros de diseño =====================
Vtrim = 30;            % m/s (solo para el MODELO LINEAL de cinemática lateral)
tau_filtro_lat = 0.30; % (lo guardo si lo usás en Simulink después; acá NO se filtra nada)

% ===================== Modelo ampliado CONSISTENTE =====================
% Estados: x_aug = [beta; p; r; phi; psi_err; y_err]
% psi_err_dot = r
% y_err_dot ≈ Vtrim*(beta + psi_err)   (porque chi_err ≈ psi_err + beta)
A_aug = zeros(6,6);
B_aug = zeros(6,2);

A_aug(1:4,1:4) = A;
B_aug(1:4,:)   = Bu;

A_aug(5,3) = 1;         % psi_err_dot = r
A_aug(6,1) = Vtrim;     % y_err_dot += Vtrim*beta
A_aug(6,5) = Vtrim;     % y_err_dot += Vtrim*psi_err

nd = size(Bv,2);
Bv_aug = [Bv; zeros(2,nd)];

fprintf('Rank ctrb(A_aug,B_aug) = %d (de 6)\n', rank(ctrb(A_aug,B_aug)));

% ===================== LQR (Bryson) =====================
beta_max = deg2rad(2);
p_max    = deg2rad(25);
r_max    = deg2rad(25);
phi_max  = deg2rad(25);
psi_max  = deg2rad(10);
y_max    = 5;             % m

Q = diag([
    1/beta_max^2
    1/p_max^2
    1/r_max^2
    1/phi_max^2
    1/psi_max^2
    1/y_max^2
]);

% Inputs normalizados (si tu Simulink satura a +/-1)
da_max = 1;
dr_max = 1;
R = diag([1/da_max^2, 1/dr_max^2]);

% Penalización extra al rudder (para no "volverse loco" con beta)
% Ajustalo si queda demasiado blando
R(2,2) = 0.1*R(2,2);
R(1,1) = 3.5*R(1,1);

K_lat = lqr(A_aug, B_aug, Q, R);
Acl   = A_aug - B_aug*K_lat;

fprintf('Polos cerrados (LQR):\n');
disp(eig(Acl).')

% ===================== Simulaciones CLOSED-LOOP =====================
tEnd = 40; dt = 0.01; t = (0:dt:tEnd).';
N = numel(t);

% --- Caso 1: Condición inicial (para ver esfuerzo y asentamiento) ---
x0 = zeros(6,1);
x0(1) = deg2rad(1);     % beta0
x0(5) = deg2rad(2);     % psi_err0
x0(6) = 5;              % y_err0 [m]

sys_cl0 = ss(Acl, zeros(6,1), eye(6), 0);
x_cl_ic = initial(sys_cl0, x0, t);
u_cl_ic = -(K_lat * x_cl_ic.').';

report_u(u_cl_ic, 'IC-only');

if plotting
plot_lat_cl(t, x_cl_ic, u_cl_ic, 'CL | Condición inicial (sin disturbios)');
end

% --- Caso 2: Gust (step en vw vía Bv) + misma condición inicial ---
w = zeros(N, nd);
t_gust = 0;            % s
vw_step = 0;           % (en unidades que interprete tu Bv)
if nd >= 1
    w(t >= t_gust, 1) = vw_step;   % vw
end

sys_cl_w = ss(Acl, Bv_aug, eye(6), zeros(6,nd));
x_cl_gust = lsim(sys_cl_w, w, t, x0);
u_cl_gust = -(K_lat * x_cl_gust.').';

report_u(u_cl_gust, sprintf('IC + vw_step=%.1f @t=%.1f', vw_step, t_gust));
if plotting
plot_lat_cl(t, x_cl_gust, u_cl_gust, sprintf('CL | IC + Gust vw step=%.1f @ %.1fs', vw_step, t_gust));
end

% ===================== Guardado =====================
save('lat_cruise.mat','K_lat','tau_filtro_lat','Vtrim');
fprintf('Saved design to lat_cruise.mat\n');

% ===================== Local functions =====================
function plot_lat_cl(t, x, u, ttl)
    beta = rad2deg(x(:,1));
    p    = rad2deg(x(:,2));
    r    = rad2deg(x(:,3));
    phi  = rad2deg(x(:,4));
    psiE = rad2deg(x(:,5));
    yE   = x(:,6);

    figure('Name',ttl);
    tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

    nexttile; plot(t,beta,'LineWidth',1.1); grid on; title('\beta [deg]');
    nexttile; plot(t,psiE,'LineWidth',1.1); grid on; title('\psi_{err} [deg]');
    nexttile; plot(t,p,'LineWidth',1.1); grid on; title('p [deg/s]');
    nexttile; plot(t,r,'LineWidth',1.1); grid on; title('r [deg/s]');
    nexttile; plot(t,phi,'LineWidth',1.1); grid on; title('\phi [deg]');
    nexttile; plot(t,yE,'LineWidth',1.1); grid on; title('y_{err} [m]');

    figure('Name',[ttl ' | Control']);
    plot(t,u(:,1),t,u(:,2),'LineWidth',1.1); grid on;
    title([ttl ' | u=-Kx']); xlabel('t [s]'); ylabel('input');
    legend('da','dr','Location','best');
end

function report_u(u, tag)
    maxDa = max(abs(u(:,1)));
    maxDr = max(abs(u(:,2)));
    fprintf('[%s] max|da|=%.4f  max|dr|=%.4f\n', tag, maxDa, maxDr);
end
