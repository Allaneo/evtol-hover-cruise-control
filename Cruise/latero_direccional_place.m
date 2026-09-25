clc; clear; close all;

load("XSWIFT_h=100_v=30.mat")

A  = mdl.lat.A;
B  = mdl.lat.B;
plotting = false;
Bu = B(:,1:2);        % [ail, rdr] actuadores
xNames = {'beta','p','r','phi'};
uNames = {'da','dr'};

%% 1) Polos y métricas de tiempo (sin t_efold)
eigVals = eig(A);

tau_s = nan(size(eigVals));   tau_s(eigVals < 0) = 1 ./ abs(eigVals(eigVals < 0));
Ts2_s = nan(size(eigVals));   Ts2_s(eigVals < 0) = 4 ./ abs(eigVals(eigVals < 0));

t_double_s = nan(size(eigVals));
t_double_s(eigVals > 0) = log(2) ./ eigVals(eigVals > 0);

T_poles = table(eigVals, tau_s, Ts2_s, t_double_s, ...
    'VariableNames', {'lambda_1_per_s','tau_s','Ts2_s','t_double_s'});

disp('=== Polos y métricas de tiempo ===');
disp(T_poles)

%% 2) Participation factors normalizados (por modo)
[V,D] = eig(A);
W = inv(V);                       % autovectores izquierdos implícitos

P = V .* (W.');                   % P(j,i) = V(j,i)*W(i,j)
Pabs  = abs(P);
Pnorm = Pabs ./ sum(Pabs,1);      % cada columna (modo) suma 1

Ptab = array2table(Pnorm, 'RowNames', xNames);
disp('=== Participation factors normalizados (filas=estados, cols=modos) ===');
disp(Ptab)

%% 3) Autoridad de control por modo (normalizada)
% En coordenadas modales: zdot = D z + (W*Bu) u
MB = W * Bu;                      % filas=modos, cols=actuadores
MBabs  = abs(MB);
MBnorm = MBabs ./ max(MBabs,[],2); % por modo, el actuador "más fuerte" queda en 1

MBtab = array2table(MBnorm, 'VariableNames', uNames);
disp('=== Efectividad modal normalizada |W*Bu| (por modo; max=1) ===');
disp(MBtab)

%% 4) Controlabilidad (para justificar place)
rc = rank(ctrb(A,Bu));
fprintf('Rank ctrb(A,Bu) = %d (de 4)\n', rc);

%% Modelo aumentado con psi para poder calcular la desviación en x
%% 5) CONTROL (place) + psi y x (cinemática fiel) + tuning para no saturar con beta0=10°
% x_aug = [beta; p; r; phi; psi; x]
% psi_dot = r
% x_dot   ≈ V*(beta + psi)

Bv = B(:,3:5);                  % [vw pw rw] para Bode (igual al longitudinal)
nd = size(Bv,2);

Vtrim = 30;                     % m/s (ajustá si corresponde)

A_aug = zeros(6,6);
B_aug = zeros(6,2);

A_aug(1:4,1:4) = A;
B_aug(1:4,:)   = Bu;

A_aug(5,3) = 1;                 % psi_dot = r
A_aug(6,1) = Vtrim;             % x_dot += V*beta
A_aug(6,5) = Vtrim;             % x_dot += V*psi

Bv_aug = [Bv; zeros(2,nd)];

%% --- Criterio: conservar los estables del airframe y corregir solo los inestables (suave) ---
lam = eig(A);
lam_st = sort(lam(real(lam)<0));                 % típicamente [-6.6; -0.93]
lam_un = sort(lam(real(lam)>0),'descend');       % típicamente [0.734; 0.018]

p_keep = lam_st(:).';                            % 2 polos a "mantener" (heurístico)
p_un_fast0   = lam_un(1);                        % +0.734
p_un_spiral0 = lam_un(2);                        % +0.018

% Requisito tuyo: x "super suave" -> polos muy cercanos a 0 (pero negativos)
p_psi = -0.20;                                   % suave
p_x   = -0.06;                                   % muy suave (si sigue agresivo, acercarlo a 0: -0.03)

% Barrido para elegir cuánto "muevo" el inestable rápido y el espiral
beta0_deg = 10;                                  % caso de diseño
p0_dps    = 0;                                   % (si querés sumar roll rate: 10–20 deg/s)

uLim = 1.0;                                      % saturación normalizada que querés respetar
tEnd = 120; dt = 0.01; t = 0:dt:tEnd;

sys_ol = ss(A_aug, zeros(6,1), eye(6), 0);

kfast_list   = [0.3 0.305 0.32 .33];  % agresividad para el modo +0.734
kspiral_list = [3.6 3.61 3.615 3.68];                                      % cuánto corregís el +0.018 (suave)

%% ==================== Barrido con criterio de factibilidad ====================
% Queremos: |da(t)|<=1 y |dr(t)|<=1 para el caso de diseño (beta0,p0) en todo t
uTol = 1e-6;                 % tolerancia numérica
feas = [];                   % filas: [kf ks maxU maxDa maxDr p_fast p_spiral score]
bestScore = -Inf;

bestK = [];
bestAcl = [];
best_pdes = [];
best = [];

for kf = kfast_list
    fprintf('kf=%.6f\n', kf);
    p_fast = -kf * p_un_fast0;

    for ks = kspiral_list
        p_spiral = -ks * p_un_spiral0;
        p_des = [p_keep, p_fast, p_spiral, p_psi, p_x];

        % Evitar polos demasiado cercanos
        if min(abs(diff(sort(p_des)))) < 1e-3
            continue
        end

        try
            K = place(A_aug, B_aug, p_des);
        catch ME
            fprintf('  place FALLÓ (kf=%.6f ks=%.6f): %s\n', kf, ks, ME.message);
            continue
        end

        Acl = A_aug - B_aug*K;

        % Chequeo estabilidad (si no es estable, ni lo evalúes)
        if any(real(eig(Acl)) >= -1e-8)
            continue
        end

        sys_cl = ss(Acl, zeros(6,1), eye(6), 0);

        x0 = zeros(6,1);
        x0(1) = deg2rad(beta0_deg);
        x0(2) = deg2rad(p0_dps);

        x_cl = initial(sys_cl, x0, t);

        u_cmd = -(K * x_cl.').';               % [da dr]
        maxDa = max(abs(u_cmd(:,1)));
        maxDr = max(abs(u_cmd(:,2)));
        maxU  = max([maxDa, maxDr]);

        % ===== Criterio principal: FACTIBILIDAD =====
        if maxU <= (uLim + uTol)
            % Criterio secundario: "más rápido" entre factibles.
            % (si no querés esto, poné score=0 y te quedás con el primero factible)
            score = abs(p_fast) + 0.2*abs(p_spiral);   % peso menor al espiral

            feas = [feas; kf ks maxU maxDa maxDr p_fast p_spiral score];

            if score > bestScore
                bestScore = score;
                best = [kf ks maxU maxDa maxDr];
                bestK = K;
                bestAcl = Acl;
                best_pdes = p_des;
            end
        end
    end
end

if isempty(feas)
    warning('No se encontró NINGÚN controlador que cumpla |u(t)|<=%.2f en el horizonte simulado. Considerá relajar polos o acortar exigencia.', uLim);

    % Como fallback: elegí el que minimiza maxU (para saber "qué tan lejos" estás)
    % (Esto es solo diagnóstico; NO cumple el criterio).
    best = []; bestK = []; bestAcl = []; best_pdes = [];
else
    disp('=== Controladores factibles (cumplen |u|<=1) ===');
    Tfeas = array2table(feas, 'VariableNames', ...
        {'kf','ks','maxU','maxDa','maxDr','p_fast','p_spiral','score'});
    % Ordenar por score descendente (más rápido primero)
    Tfeas = sortrows(Tfeas, 'score', 'descend');
    disp(Tfeas(1:min(15,height(Tfeas)),:));

    disp('=== Mejor candidato (FACTIBLE + más rápido por score) ===');
    disp(array2table(best, 'VariableNames', {'kfast','kspiral','maxU','maxDa','maxDr'}));
    disp('p_des ='); disp(best_pdes);
    disp('Polos cerrados ='); disp(eig(bestAcl).');
end

K_lat = bestK;
Acl   = bestAcl;


% Report final del caso de diseño
sys_cl = ss(Acl, zeros(6,1), eye(6), 0);
x0 = zeros(6,1); x0(1)=deg2rad(beta0_deg); x0(2)=deg2rad(p0_dps);
x_ol = initial(sys_ol, x0, t);
x_cl = initial(sys_cl, x0, t);

u_cmd = -(K_lat * x_cl.').';
da_cmd = u_cmd(:,1);
dr_cmd = u_cmd(:,2);

fprintf('Caso diseño beta0=%.1f deg: max|da|=%.3f, max|dr|=%.3f (lim=%.1f)\n', ...
    beta0_deg, max(abs(da_cmd)), max(abs(dr_cmd)), uLim);

% Plots clave (mantengo tu estilo: OL vs CL + ambos controles juntos)
if plotting
plot_case_lat_single(t, x_ol, x_cl, da_cmd, dr_cmd, ...
    sprintf('Perturbación inicial: beta(0)=%.1f° y p(0)=%.1f°/s', beta0_deg, p0_dps));

% Bode demanda de control (igual que longitudinal)
ema_bw_hz = 10; w_ema = 2*pi*ema_bw_hz;
sys_u_v = ss(Acl, Bv_aug, -K_lat, zeros(2,nd));
plot_bode_control_demand_lat(sys_u_v, w_ema);
end


%% ===================== Local functions =====================
function plot_case_lat_single(t, x_ol, x_cl, da_cmd, dr_cmd, case_title)
    % x_aug = [beta p r phi psi x]
    beta_ol = rad2deg(x_ol(:,1));   beta_cl = rad2deg(x_cl(:,1));
    p_ol    = rad2deg(x_ol(:,2));   p_cl    = rad2deg(x_cl(:,2));
    r_ol    = rad2deg(x_ol(:,3));   r_cl    = rad2deg(x_cl(:,3));
    phi_ol  = rad2deg(x_ol(:,4));   phi_cl  = rad2deg(x_cl(:,4));
    psi_ol  = rad2deg(x_ol(:,5));   psi_cl  = rad2deg(x_cl(:,5));
    x_ol_m  = x_ol(:,6);            x_cl_m  = x_cl(:,6);

    limitesy = [-15 15];
    figure; plot(t, beta_ol,'--', t, beta_cl,'-'); grid on; ylim([limitesy]);
    title([case_title ' | beta(t)']); xlabel('s'); ylabel('deg'); legend('Open-loop','Closed-loop');

    figure; plot(t, p_ol,'--', t, p_cl,'-'); grid on; ylim([limitesy]);
    title([case_title ' | p(t)']); xlabel('s'); ylabel('deg/s'); legend('Open-loop','Closed-loop');

    figure; plot(t, r_ol,'--', t, r_cl,'-'); grid on; ylim([limitesy]);
    title([case_title ' | r(t)']); xlabel('s'); ylabel('deg/s'); legend('Open-loop','Closed-loop');

    figure; plot(t, phi_ol,'--', t, phi_cl,'-'); grid on; ylim([limitesy]);
    title([case_title ' | phi(t)']); xlabel('s'); ylabel('deg'); legend('Open-loop','Closed-loop');

    figure; plot(t, psi_ol,'--', t, psi_cl,'-'); grid on; ylim([limitesy]);
    title([case_title ' | psi(t)']); xlabel('s'); ylabel('deg'); legend('Open-loop','Closed-loop');

    figure; plot(t, x_ol_m,'--', t, x_cl_m,'-'); grid on; ylim([limitesy]);
    title([case_title ' | x(t) (desvío lateral)']); xlabel('s'); ylabel('m'); legend('Open-loop','Closed-loop');

    figure; plot(t, da_cmd, 'LineWidth', 1.1); hold on; grid on;
    plot(t, dr_cmd, 'LineWidth', 1.1);
    title([case_title ' | esfuerzo de control']); xlabel('s'); ylabel('input units');
    legend('da','dr','Location','best');
end

function plot_bode_control_demand_lat(sys_u_v, w_ema)
    w = logspace(-3, 2.5, 1200);           % rad/s
    G = freqresp(sys_u_v, w);              % 2 x nd x N
    Mag = abs(G);
    nd = size(G,2);

    vNames = {'vw','pw','rw'};

    figure;
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

    nexttile; hold on; grid on; set(gca,'XScale','log');
    title('Demanda de control: |da / v_i|');
    xlabel('\omega [rad/s]'); ylabel('Magnitud [dB]');
    for i = 1:nd
        plot(w, 20*log10(squeeze(Mag(1,i,:)) + 1e-12), 'LineWidth', 1.1);
    end
    xline(w_ema, ':', sprintf('\\omega_{ema}=%.3g', w_ema), 'LineWidth', 1.0);
    legend(vNames, 'Location','eastoutside');

    nexttile; hold on; grid on; set(gca,'XScale','log');
    title('Demanda de control: |dr / v_i|');
    xlabel('\omega [rad/s]'); ylabel('Magnitud [dB]');
    for i = 1:nd
        plot(w, 20*log10(squeeze(Mag(2,i,:)) + 1e-12), 'LineWidth', 1.1);
    end
    xline(w_ema, ':', sprintf('\\omega_{ema}=%.3g', w_ema), 'LineWidth', 1.0);
    legend(vNames, 'Location','eastoutside');
end
tau_filtro_lat = 0.2;
save('lat_cruise.mat','K_lat','tau_filtro_lat');
fprintf('Saved design to lat_cruise.mat\n');