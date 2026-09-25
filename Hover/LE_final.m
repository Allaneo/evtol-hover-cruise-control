% Lazo Externo Final

clear; close all; clc;

% --- Datos
s = tf('s');
Gp = 9.3034/(s + 1.476);

% Especificaciones
f_bw = 40;                 % Hz (ancho de banda deseado aproximado)
omega_d = 2*pi*f_bw;     % rad/s (frecuencia amortiguada aproximada)
zeta = 0.7;               % amortiguamiento deseado

% Cálculos auxiliares: omega_n y s_des
omega_n = omega_d / sqrt(1 - zeta^2);
sigma = -zeta * omega_n;            % parte real deseada
s_des = sigma + 1i*omega_d;         % punto deseado en el plano

fprintf('Objetivo: zeta=%.3f, f=%.2f Hz -> omega_n=%.3f rad/s, s_des = %.3f %+.3fi\n', ...
    zeta, f_bw, omega_n, real(s_des), imag(s_des));

% --- Encontrar z (cero del PI) mediante la condicion de angulo
% C(s) = K*(s+z)/s. Polos abiertos: 0, -1.476. Cero: -z.
% Condicion de angulo: angle(s_des + z) - angle(s_des) - angle(s_des + 1.476) = pi  (rad)

angle_residual = @(zvar) wrapToPi(angle(s_des + zvar) - (pi + angle(s_des) + angle(s_des + 1.476)));

% fzero necesita un real scalar; buscamos z alrededor de omega_d (buen init)
z0 = omega_d;
options = optimset('Display','off');
[z_sol, fval, exitflag] = fzero(angle_residual, z0, options);

if exitflag <= 0
    error('No se encontro solucion para z. Intenta cambiar el punto inicial.');
end

z = real(z_sol);   % z debe ser real positivo
fprintf('Cero (z) encontrado: z = %.6f rad/s  -> fz = %.4f Hz\n', z, z/(2*pi));

% --- Calculo de K por condicion de magnitud
num_val = 9.3034 * abs(s_des + z);
den_val = abs(s_des) * abs(s_des + 1.476);
L_mag_at_sdes_noK = num_val / den_val;   % magnitud de L(s_des) sin K
K = 1 / L_mag_at_sdes_noK;

fprintf('Magnitudes en s_des: |s_des+z|=%.6f, |s_des|=%.6f, |s_des+1.476|=%.6f\n', ...
    abs(s_des+z), abs(s_des), abs(s_des+1.476));
fprintf('K calculada por condicion de magnitud = %.6f\n', K);

% Derivar Kp y Ki del PI: C(s)=K*(s+z)/s = K + K*z*(1/s) => Kp = K, Ki = K*z
Kp = K;
Ki = K * z;
fprintf('Controlador PI: Kp = %.6f, Ki = %.6f\n', Kp, Ki);

% --- Construir controlador y lazos
C = K * (s + z) / s;      % controlador completo
L = C * Gp;               % lazo abierto con K incluido
T = feedback(L, 1);       % lazo cerrado
S = feedback(1, L);       % funcion sensibilidad

% --- Verificar polos cerrados
p_cl = pole(T);
fprintf('Polos cerrados (T):\n'); disp(p_cl);

% --- Graficos solicitados: 1) Root Locus con s_des marcado y sgrid zeta
figure('Name','Root Locus y s_{des}','Color','w','Units','normalized','Position',[0.05 0.55 0.4 0.35]);
% Para hacer rlocus variando K, dibujamos rlocus de (s+z)/s * Gp (sin K)
C_noK = (s + z) / s;
rlocus(C_noK * Gp); hold on; grid on;
% marcar polos abiertos y ceros
plot(0,0,'rx','MarkerSize',10,'LineWidth',1.5); % polo en 0
plot(-1.476,0,'rx','MarkerSize',10,'LineWidth',1.5); % polo planta
plot(-z,0,'bo','MarkerSize',8,'LineWidth',1.5); % cero del controlador
% marcar s_des
plot(real(s_des), imag(s_des), 'kp', 'MarkerSize',12, 'MarkerFaceColor','y');
text(real(s_des), imag(s_des), sprintf('  s_{des}=%.1f%+.1fj', real(s_des), imag(s_des))); 
% sgrid para amortiguamiento
sgrid(zeta,[]);
title('Root Locus de (s+z)/s * G_p(s) con s_{des} marcado');

% --- En el mismo grafico dibujar vectores desde polos/ceros hasta s_des y mostrar angulos
figure('Name','Vectores y angulos en s_{des}','Color','w','Units','normalized','Position',[0.5 0.55 0.4 0.35]);
plot(real(s_des), imag(s_des), 'kp', 'MarkerSize',12, 'MarkerFaceColor','y'); hold on; grid on;
% ubicaciones
pole0 = 0 + 0i;
pole1 = -1.476 + 0i;
zeroC = -z + 0i;
% dibujar vectores
plot([real(zeroC) real(s_des)],[imag(zeroC) imag(s_des)],'b-','LineWidth',1.2);
plot([real(pole0) real(s_des)],[imag(pole0) imag(s_des)],'r-','LineWidth',1.2);
plot([real(pole1) real(s_des)],[imag(pole1) imag(s_des)],'r-','LineWidth',1.2);

% --- Bode y margen del L (con K)
figure('Name','Bode de L (con K)','Color','w','Units','normalized','Position',[0.5 0.05 0.4 0.35]);
[magL,phL,wout] = bode(L, {1e-1,1e5});
magL = squeeze(magL); phL = squeeze(phL);
subplot(2,1,1);
semilogx(wout,20*log10(magL)); grid on; hold on; xlabel('rad/s'); ylabel('Mag (dB)');
% marcar wc objetivo
wc_target = omega_d; xline(wc_target,'k--','LineWidth',1.2); text(wc_target, -10, 'wc target');
subplot(2,1,2);
semilogx(wout,phL); grid on; xlabel('rad/s'); ylabel('Phase (deg)');

% --- Bode de T y S
figure('Name','Bode T y S','Color','w');
subplot(2,1,1); bode(T); grid on; title('Bode de T(s)');
subplot(2,1,2); bode(S); grid on; title('Bode de S(s)');

% --- Respuesta al escalon del lazo cerrado
figure('Name','Step response closed-loop','Color','w');
step(T); grid on; title('Step response de lazo cerrado T(s)');

% --- Mostrar numericos finales
fprintf('\nResumen numerico:\n');
fprintf('z = %.6f rad/s  (fz = %.4f Hz)\n', z, z/(2*pi));
fprintf('K = %.6f, Kp = %.6f, Ki = %.6f\n', K, Kp, Ki);




%% ---------------------- Parámetros extraídos del paper -----------------------
alpha_rpm = 1.18e-7;      % [Nm / (RPM^2)] - estimado a partir de T del paper.
omega_nom_rpm = 2400;     % RPM - punto de operación del paper
Kv = 275;       % RPM per Volt (dato)

% Conversión y constantes
rpm2rad = 2*pi/60;        % multiplicador RPM -> rad/s : omega_rad = RPM * 2*pi/60

% Convertir alpha (Nm/(RPM^2)) a kQ (Nm/(rad/s)^2) para usar T = kQ * omega_rad^2
kQ = alpha_rpm * rpm2rad^-2;    % [Nm / (rad/s)^2]  because RPM = (60/(2*pi))*omega_rad

fprintf('Derived aerodynamic torque coefficient (kQ) = %.4e N/(rad/s)^2\n', kQ);

% Punto de operacion en rad/s
omega_nom = omega_nom_rpm * rpm2rad;   % [rad/s]
fprintf('omega_nom = %.3f rad/s (%.0f RPM)\n', omega_nom, omega_nom_rpm);

% Motor constants: Kv -> Ke -> Kt (SI units)
Kv_rad_per_V = Kv * rpm2rad; % [rad/s per V]
Ke = 1 / Kv_rad_per_V;                  % V/(rad/s)
Kt = 1*Ke;                                % approximacion SI: Kt (Nm/A) ~= Ke (V/(rad/s))
fprintf('Kv (rad/s/V) = %.3f => Ke = %.5f V/(rad/s) => Kt = %.5f Nm/A (approx)\n', Kv_rad_per_V, Ke, Kt);

%% ---------------------- Suposiciones / estimaciones (anotadas) ----------------
% INERCIA J: Hacer medicion si se puede. Pongo un valor razonable para un motor+helice 18":
% Si tenes valor experimental reemplazarlo aqui.
J_motor = 0.01*(0.432^2)*2;          % kg*m^2  
b = 1e-4;           % N*m/(rad/s) roce viscoso
fprintf('Usando J = %.4g kg*m^2 (estimado), b = %.4g N*m/(rad/s) (estimado)\n', J_motor, b);

%% ---------------------- Linearizacion del torque aerodinamico ----------------
% T_aero (no lineal) en RPM: T = alpha_rpm * RPM^2
% Linealizacion alrededor de omega_nom (en rad/s):
% dT/domega_rad = 2 * kQ * omega_nom   (porque T = kQ * omega_rad^2)
dTaero_domega = 2 * kQ * omega_nom;   % [N*m / (rad/s)]   (slope around omega_nom)
fprintf('Linearized aerodynamic torque slope at omega_nom: dT/domega = %.4e N*m/(rad/s)\n', dTaero_domega);

% Calculo del termino D = b + dT/domega  (amortiguamiento de sistema linealizado)
D = b + dTaero_domega;      
tau_m = J_motor / D;   % tiempo caracteristico mecanico
fprintf('Amortiguamiento efectivo D = %.4e  -> tau_m = J/D = %.4g s\n', D, tau_m);
Gp_real = tf(Kt, [J_motor, b]);


save('outer_controller_design.mat','J_motor','b','kQ','Ke','Kp','Ki','Kt');
fprintf('Saved design to outer_controller_design.mat\n');
