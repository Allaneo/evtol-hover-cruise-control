% Diseño automático del controlador de VELOCIDAD usando parametrización afín.

clear; close all; clc;
s = tf('s');

%% ----------------- 0) Especificaciones / parámetros -----------------
m=13;
J_xz = 2*(0.5)^2*m/2;
% BANDWIDTH objetivo para el lazo velocidad (desde el análisis en cascada)
f_vangular = 4.0;           % Hz 
omega_v = 2*pi*f_vangular;

fprintf('Design specs: f_v = %.3f Hz -> wn_v = %.3f rad/s', ...
    f_vangular, omega_v);

Gvangular = 1/(J_xz*s);
%% ----------------- 1) Polinomio objetivo -------------
n_v = 1;                                 % orden Bessel
[num,den] = besself(n_v, omega_v);     

%% ----------------- 2) Parametrizacion afín----
Fq = tf(num,den);
Kvangular = minreal((Fq/(1 - Fq)) * 1/Gvangular);

L = minreal(Gvangular*Kvangular);
%bode(Fq); grid on
[Kvangular_num, Kvangular_den] = tfdata(Kvangular, 'v');

save('angular_speed_controller_design.mat','Kvangular_num','Kvangular_den','Gvangular');
fprintf('Saved design to angular_speed_controller_design.mat\n');
