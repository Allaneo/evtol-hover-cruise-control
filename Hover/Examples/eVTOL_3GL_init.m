clear; clc

go      = 9.80667;  % aceleracion de la gravedad
mass    = 13;       % masa    
inertia = 2.4;      % momento de inercia (CBE)
f_arm   = 0.7;      % distancia de los rotores al CG (CBE)   
SCx     = 0.1;      % area frontal x CD (CBE)
SCz     = 0.3;      % area frontal x CD (CBE)
rho     = 1.24;     % densidad del aire
w_esc   = 10;       % anchod e banda de los ESC 

% ganancias
k_T = mass*go*1.8/2; % T = k_T n^2
k_M = f_arm/inertia; % M = k_M (T1 - T2)
% velocidad de giro (normalizada) inicial
no  = sqrt(mass*go/(2*k_T));

