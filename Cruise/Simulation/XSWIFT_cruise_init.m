% ===================================================================
% CONTROL Y GUIADO  
%                                         Departamento de Aeronáutica
%                                              Facultad de Ingeniería
%                                                            U.N.L.P.
% ===================================================================
% 
% ___________________________________________________________________
clear;
clc
load('XSWIFT_data');
load('XSWIFT_sim' );
load('XSWIFT_nav' );

data.clr = zeros(size(data.clr));


h_offset = 5; % altura de la pasada

%% Condiciones másicas
mass_frac = 1;
x_cg      = 0;
x_ref     = 0;

disable_measurement_noise = false;   

% Condiciones Atmosféricas
wind.heading = 45;  % [°] en relación al norte
wind.speed   = 4.0;   % m/s                         
wind.seeds   = [23341 23342 23343 23344];

%% Datos adicionales
masa    = data.mass.zfw + (data.mass.max_takeoff - data.mass.zfw) * mass_frac;
inercia = data.mass.inertia * masa / data.mass.max_takeoff;

XSWIFT_actuators;
XSWIFT_sensors;

%% Registro de datos 
T_rec = 1/20;
N_rec = 1024^2;
rec_name = 'sim';
auto_save = false

% FlightGear
timeSince1970 = 1.3201e+09;
Tc = 1/25; % muestreo para la animación

r2d = 180/pi;
d2r = pi/180;
go  = 9.80566;


%%
init.vel     = 30; % [m/s]
init.thr     = 0.7;
init.elv     = 0.0200;
init.alt     = 100;
init.alpha   = 0.01;
init.lla     = [-35.3519 -57.2924 init.alt];
init.heading = 90;
init.vb      = [cos(init.alpha) 0 sin(init.alpha)]*init.vel;
init.eul     = [0 init.alpha init.heading*d2r];
init.wb      = [0 0 0];


engine.bw = 5;
engine.A  = -engine.bw;
engine.B  =  engine.bw;
engine.C  =  masa*go/5;
