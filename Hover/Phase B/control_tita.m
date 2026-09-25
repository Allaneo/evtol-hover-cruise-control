% Diseño automático del controlador de POSICIÓN usando enfoque polinomial.

clear; close all; clc;
s = tf('s');

%% ----------------- 0) Especificaciones / parámetros -----------------
% BANDWIDTH objetivo para el lazo velocidad (desde el análisis en cascada)
f_tita = 4;           % Hz
omega_tita = 2*pi*f_tita;
l_palanca = 0.4;
m=13;
J_xz = (m/12)*(1^2+1^2)

fprintf('Design specs: f_p = %.3f Hz -> wn_p = %.3f rad/s \n', ...
    f_tita, omega_tita);

Gtita =1/(J_xz*s^2);

%% ----------------- 1) Enfoque Polinomial con Bessel ----
[~,den] = besself(3,omega_tita);
L1 = den(1);
L0 = den(2);
P1 = den(3);
P0 = den(4);
Ktita_num = [P1,P0];
Ktita_den = [L1,L0];
Ktita = (P1*s+P0)/(L1*s+L0)

L = Gtita*Ktita;
T_Bessel = feedback(L,1);

%bode(T_Bessel); grid on;

save('tita_controller_design.mat','Ktita_num','Ktita_den','Gtita',"l_palanca",'J_xz');
fprintf('Saved design to tita_controller_design.mat\n');
