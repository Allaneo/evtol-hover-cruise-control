%% Plotter_resultados.m
% Workspace esperado:
%   rec_cmnd   : [elevador, alerones, rudder, thrust] (NORMALIZADOS [-1,1])
%   rec_flight : [alfa, beta, TAS, height]
%   rec_wind   : (opcional) [Vwx Vwy Vwz] o [Vwx Vwy Vwz Wwx Wwy Wwz] o N-col
%   rec_ins    : (opcional) INS bus aplanado: [sfc(3) pqr(3) eul(3) Xned(3) Vned(3)] -> psi=eul(3)
%
% NOTA: Cross-track se calcula con la cinemática discutida:
%   e_dot = V * (beta + psi_err)
%   psi_err = wrapToPi(psi - psi_ref0)
%   e = integral(e_dot dt)

clearvars -except rec_cmnd rec_flight rec_wind rec_ins

%% ======= Config =======
simDuration = 85;            % [s] si no hay time en logs
n0 = 30;                     % muestras iniciales para definir referencias (psi_ref0, h_ref0)
SATURATE_COMMANDS = true;    % clip a [-1,1] para visualizar saturaciones

%% ======= Cargar señales =======
assert(exist('rec_cmnd','var')==1,   'Falta rec_cmnd en el workspace.');
assert(exist('rec_flight','var')==1, 'Falta rec_flight en el workspace.');

[cmnd,  t_cmnd_raw] = unwrapSignal(rec_cmnd);
[flight,t_flt_raw]  = unwrapSignal(rec_flight);

t_cmnd = makeTime(t_cmnd_raw, size(cmnd,1),   simDuration);
t_flt  = makeTime(t_flt_raw,  size(flight,1), simDuration);

hasWind = (exist('rec_wind','var')==1) && ~isempty(rec_wind);
if hasWind
    [windRaw, t_wind_raw] = unwrapSignal(rec_wind);
    t_wind = makeTime(t_wind_raw, size(windRaw,1), simDuration);
else
    windRaw = [];
    t_wind  = [];
end

hasINS = (exist('rec_ins','var')==1) && ~isempty(rec_ins);
if hasINS
    [insRaw, t_ins_raw] = unwrapSignal(rec_ins);
    t_ins = makeTime(t_ins_raw, size(insRaw,1), simDuration);
else
    insRaw = [];
    t_ins  = [];
end

fprintf('\n--- Sizes ---\n');
fprintf('rec_cmnd   : %dx%d\n', size(cmnd,1),  size(cmnd,2));
fprintf('rec_flight : %dx%d\n', size(flight,1),size(flight,2));
if hasWind, fprintf('rec_wind   : %dx%d\n', size(windRaw,1),size(windRaw,2));
else,       fprintf('rec_wind   : (no disponible)\n'); end
if hasINS,  fprintf('rec_ins    : %dx%d\n', size(insRaw,1), size(insRaw,2));
else,       fprintf('rec_ins    : (no disponible)\n'); end
fprintf('------------\n\n');

%% ======= Validaciones mínimas =======
assert(size(cmnd,2)   >= 4, 'rec_cmnd debe tener >=4 columnas: [elev, ail, rud, thrust].');
assert(size(flight,2) >= 4, 'rec_flight debe tener >=4 columnas: [alfa, beta, TAS, height].');

%% ======= Separar señales =======
% COMANDOS: unitless, normalizados [-1,1]
elev  = cmnd(:,1); elev  = elev(:);
ail   = cmnd(:,2); ail   = ail(:);
rud   = cmnd(:,3); rud   = rud(:);
thrst = cmnd(:,4); thrst = thrst(:);

[elev,ail,rud,thrst] = checkAndClipCommands(elev,ail,rud,thrst,SATURATE_COMMANDS);

% VUELO
alfa_raw = flight(:,1); alfa_raw = alfa_raw(:);
beta_raw = flight(:,2); beta_raw = beta_raw(:);
TAS      = flight(:,3); TAS      = TAS(:);
h        = flight(:,4); h        = h(:);

% Para plots, convierto a deg si parece rad (solo display)
[alfa_deg, aUnit] = autoAngleToDeg(alfa_raw);
[beta_deg, bUnit] = autoAngleToDeg(beta_raw);

%% ======= Error de altura (simple) =======
nUse_h = min(n0, numel(h));
h_ref0 = 100;     % referencia: altura inicial (promedio)
e_h    = h - h_ref0;                       % error de altura

%% ======= Cross-track desde cinemática: e_dot = V*(beta + psi_err) =======
e_y     = [];
e_dot   = [];
psi_err = [];
psi_flt = [];

if hasINS && ~isempty(insRaw)
    % Extraer psi desde INS (eul(3))
    psi_ins = extractPsiFromINS(insRaw);     % [rad], envuelto

    % Interpolar psi a tiempo de flight
    psi_flt = interp1(t_ins, psi_ins, t_flt, 'linear', 'extrap');

    % Referencia de rumbo: heading inicial (promedio)
    nUse_psi = min(n0, numel(psi_flt));
    psi_ref0 = deg2rad(90);

    psi_err = wrapToPi_local(psi_flt - psi_ref0);

    % Beta en rad para la cinemática (blindado por si vino en deg)
    beta_rad = beta_raw;
    if max(abs(beta_rad),[],'omitnan') > (2*pi + 0.5)   % si parece deg
        beta_rad = deg2rad(beta_rad);
    end

    % V para cinemática: TAS (según lo que venías usando)
    V = TAS;

    % e_dot = V*(beta + psi_err)
    e_dot = V .* (beta_rad + psi_err);

    % Integración sobre t_flt (no invento Ts)
    e_y = cumtrapz(t_flt, e_dot);
end

%% ======= PLOTS: Commands =======
figure('Name','Commands','Color','w');
tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

nexttile;
plot(t_cmnd, elev, t_cmnd, ail, t_cmnd, rud, 'LineWidth', 1.0);
grid on; xlabel('t [s]');
ylabel('Command [unitless]');
legend('Elevator','Aileron','Rudder','Location','best');
title('Control Commands (normalized)');
ylim([-1.1 1.1]);
yline( 1,'--'); yline(-1,'--');

nexttile;
plot(t_cmnd, thrst, 'LineWidth', 1.0);
grid on; xlabel('t [s]');
ylabel('Thrust [units]');
title('Thrust Command');

%% ======= PLOTS: Flight =======
figure('Name','Flight','Color','w');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

nexttile; plot(t_flt, alfa_deg, 'LineWidth', 1.0); grid on; xlabel('t [s]'); ylabel(sprintf('\\alpha [%s]',aUnit)); title('\alpha');
nexttile; plot(t_flt, beta_deg, 'LineWidth', 1.0); grid on; xlabel('t [s]'); ylabel(sprintf('\\beta [%s]',bUnit)); title('\beta');
nexttile; plot(t_flt, TAS,      'LineWidth', 1.0); grid on; xlabel('t [s]'); ylabel('TAS [m/s]'); title('True Airspeed');
nexttile; plot(t_flt, h,        'LineWidth', 1.0); grid on; xlabel('t [s]'); ylabel('h [m]'); title('Altitude');

%% ======= PLOTS: Errores (altura + cross-track) =======
figure('Name','Errors','Color','w');
tiledlayout(2,2,'TileSpacing','compact','Padding','compact');

% Altura
nexttile;
plot(t_flt, e_h, 'LineWidth', 1.1);
grid on; xlabel('t [s]'); ylabel('e_h [m]');
title('Altitude error: e_h = h - h_{ref0}');
yline(0,'--');

% Cross-track integrado
nexttile;
if ~isempty(e_y)
    plot(t_flt, e_y, 'LineWidth', 1.1);
    grid on; xlabel('t [s]'); ylabel('e_y [m]');
    title('Cross-track: e_y = \int V(\beta+\psi_{err}) dt');
    yline(0,'--');
else
    axis off;
    text(0,0.5,'No rec\_ins -> no se puede calcular \psi_{err} -> no cross-track', 'FontSize', 11);
end

% Variables usadas: psi_err
nexttile;
if ~isempty(psi_err)
    plot(t_flt, rad2deg(psi_err), 'LineWidth', 1.1);
    grid on; xlabel('t [s]'); ylabel('\psi_{err} [deg]');
    title('\psi_{err} = wrapToPi(\psi-\psi_{ref0})');
    yline(0,'--');
else
    axis off;
end

% Variables usadas: beta (display deg) y V (TAS)
nexttile;
yyaxis left;
plot(t_flt, beta_deg, 'LineWidth', 1.0);
ylabel('\beta [deg]');
yyaxis right;
plot(t_flt, TAS, 'LineWidth', 1.0);
ylabel('TAS [m/s]');
grid on; xlabel('t [s]');
title('Variables usadas en \dot e_y = V(\beta+\psi_{err})');

%% ======= PLOTS: Wind (simple, sin "expected wind" raro) =======
if hasWind && ~isempty(windRaw)
    figure('Name','Wind','Color','w');

    [Nw,Mw] = size(windRaw);
    tiledlayout(2,1,'TileSpacing','compact','Padding','compact');

    nexttile;
    if Mw >= 3
        plot(t_wind, windRaw(:,1), t_wind, windRaw(:,2), t_wind, windRaw(:,3), 'LineWidth', 1.0);
        grid on; xlabel('t [s]'); ylabel('V_{wind} [m/s]');
        legend('Vwx','Vwy','Vwz','Location','best');
        title('Wind velocity components (cols 1:3)');
    else
        plot(t_wind, windRaw, 'LineWidth', 1.0);
        grid on; xlabel('t [s]'); ylabel('wind (?)');
        title('Wind signal (raw)');
    end

    nexttile;
    if Mw >= 6
        plot(t_wind, windRaw(:,4), t_wind, windRaw(:,5), t_wind, windRaw(:,6), 'LineWidth', 1.0);
        grid on; xlabel('t [s]'); ylabel('\omega_{wind} [rad/s]');
        legend('\omega_x','\omega_y','\omega_z','Location','best');
        title('Wind angular rates (cols 4:6)');
    else
        Vmag = vecnorm(windRaw(:,1:min(3,Mw)),2,2);
        plot(t_wind, Vmag, 'LineWidth', 1.0);
        grid on; xlabel('t [s]'); ylabel('|V_{wind}| [m/s]');
        title('Wind speed magnitude');
    end
end

%% ================== Helpers ==================

function [x, tx] = unwrapSignal(s)
    tx = [];
    if isa(s,'timeseries')
        tx = s.Time(:);
        x  = squeeze(s.Data);
        x  = enforce2D(x);
        return;
    end
    if isstruct(s) && isfield(s,'time') && isfield(s,'signals')
        tx = s.time(:);
        x  = squeeze(s.signals.values);
        x  = enforce2D(x);
        return;
    end
    x = enforce2D(s);
end

function x = enforce2D(x)
    if isempty(x), return; end
    if ~isnumeric(x)
        error('La señal no es numérica (no puedo plotearla).');
    end
    x = squeeze(x);
    if size(x,1) < size(x,2) && size(x,1) <= 10 && size(x,2) > 50
        x = x.'; % probable transpuesta
    end
    if isvector(x), x = x(:); end
end

function t = makeTime(t_in, N, simDuration)
    if ~isempty(t_in) && isnumeric(t_in) && numel(t_in)==N
        t = t_in(:);
        return;
    end
    t = linspace(0, simDuration, N).';
end

function [angDeg, unitStr] = autoAngleToDeg(ang)
    ang = ang(:);
    amax = max(abs(ang(~isnan(ang))));
    if isempty(amax), amax = 0; end
    if amax <= 3.5
        angDeg  = rad2deg(ang);
        unitStr = 'deg';
    else
        angDeg  = ang;
        unitStr = 'deg?';
    end
end

function [elev,ail,rud,thrst] = checkAndClipCommands(elev,ail,rud,thrst,doClip)
    maxAbs = max([max(abs(elev)), max(abs(ail)), max(abs(rud))], [], 'omitnan');
    if maxAbs > 1.02
        fprintf('[Commands] OJO: comandos fuera de [-1,1]. max|cmd|=%.3f\n', maxAbs);
        if doClip
            elev  = max(min(elev,  1), -1);
            ail   = max(min(ail,   1), -1);
            rud   = max(min(rud,   1), -1);
            fprintf('[Commands] Se aplicó saturación a [-1,1] para visualizar.\n');
        else
            fprintf('[Commands] No se saturó (SATURATE_COMMANDS=false).\n');
        end
    end
    % thrust no necesariamente es [-1,1]
end

function psi = extractPsiFromINS(insRaw)
    % Espera INS "aplanado" con eul en cols 7:9 y psi=eul(3)=col 9
    [~,M] = size(insRaw);
    if M < 9
        error('rec_ins no tiene suficientes columnas para extraer eul(3)=psi (col 9).');
    end

    psi = insRaw(:,9); % eul(3)

    % Si viene en grados (valores tipo 180), pasar a rad
    if max(abs(psi),[],'omitnan') > 2*pi + 0.5
        psi = deg2rad(psi);
    end

    psi = wrapToPi_local(psi);
end

function ang = wrapToPi_local(ang)
    ang = mod(ang + pi, 2*pi) - pi;
end
