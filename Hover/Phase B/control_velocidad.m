% Diseño automático del controlador de VELOCIDAD usando parametrización afín.

clear; close all; clc;
s = tf('s');

%% ----------------- 0) Especificaciones / parámetros -----------------
m = 13;
n = 4;                   % numero de rotores
omega0_rpm = 2400;       % rpm punto de trabajo
rpm2rad = 2*pi/60;
omega0 = omega0_rpm * rpm2rad;   % rad/s


KF = 9.81*m/(n*omega0^2);   % N/(rad/s)^2


% BANDWIDTH objetivo para el lazo velocidad (desde el análisis en cascada)
f_v = .8;           % Hz 
omega_v = 2*pi*f_v;

fprintf('Design specs: f_v = %.3f Hz -> wn_v = %.3f rad/s', ...
    f_v, omega_v);

Gv = 1/(m*s);
%% ----------------- 1) Polinomio objetivo -------------
n_v = 1;                                 % orden Bessel
[~,den] = besself(n_v, omega_v);     

%% ----------------- 2) Parametrizacion afín----
%num = den(2:end); % grado relativo 1!
num=den(end);
Fq = tf(num,den);
Kv = minreal((Fq/(1 - Fq)) * 1/Gv)

L = minreal(Gv*Kv);

%bode(Fq); grid on
[Kv_num, Kv_den] = tfdata(Kv, 'v');

save('speed_controller_design.mat','Kv_num','Kv_den','Gv','m','KF');
fprintf('Saved design to speed_controller_design.mat\n');
