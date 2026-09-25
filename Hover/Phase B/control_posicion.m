% Diseño automático del controlador de POSICIÓN usando enfoque polinomial.

clear; close all; clc;
s = tf('s');

%% ----------------- 0) Especificaciones / parámetros -----------------
% BANDWIDTH objetivo para el lazo velocidad (desde el análisis en cascada)
f_p = .2;           % Hz
omega_p = 2*pi*f_p;

fprintf('Design specs: f_p = %.3f Hz -> wn_p = %.3f rad/s', ...
    f_p, omega_p);

Gp = 1/s;
%% ----------------- 1) Polinomio objetivo (Óptimo ITAE) -------------

obj_3 = 1;
obj_2 = 1.75*omega_p;
obj_1 = 2.15*omega_p^2;
obj_0 = omega_p^3;

%% ----------------- 2) Enfoque Polinomial con ITAE----
L1 = obj_3;
L0 = obj_2;
P1 = obj_1;
P0 = obj_0;

Kp = (P1*s+P0)/(L1*s+L0);

L = Gp*Kp;
T_ITAE = feedback(L,1);

%% ----------------- 3) Enfoque Polinomial con Bessel ----
[~,den] = besself(2,omega_p);
L1 = den(1);
L0 = den(2);
P1 = 0;
P0 = den(3);
K_pos_num = [P1,P0];
K_pos_den = [L1,L0];
Kp = (P1*s+P0)/(L1*s+L0)

L = Gp*Kp;
T_Bessel = feedback(L,1);

%bode(T_ITAE); hold on;
%bode(T_Bessel); grid on; legend on;

save('position_controller_design.mat','K_pos_num','K_pos_den','Gp');
fprintf('Saved design to position_controller_design.mat\n');
