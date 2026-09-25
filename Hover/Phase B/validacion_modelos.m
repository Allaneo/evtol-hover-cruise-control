clc; clear; clf;

% implementación analógica del lazo interno
La = 0.0004;
Ra = 0.066;
G = tf(1, [La, Ra]);

K=3.2;
Tcontrolador=10/2512;
numC = [Tcontrolador,1];
denC = [K*Tcontrolador,1];
C = K*tf(numC,denC);

L = G*C;

R1 = 10e3;
R2 = 22e3;
C1 = 0.39e-6;
C2 = 0.56e-6;

R1C1 = R1*C1;
R2C2 = R2*C2;

T=R1C1;
alphaT=R2C2;
K=(R2+R1)*C1/(R2*C2);
L_analogico =G*K*tf([1,1/T],[1,1/alphaT]);
%margin(L); grid on; hold on; bode(L_analogico);legend;

% implementación digital del lazo interno
[GM,PM,Wcg,Wcp] = margin(L); 
omega_cruce = Wcp;
frec_cruce  = omega_cruce/(2*pi);
ts = 1/(10*frec_cruce);
opt = c2dOptions('Method','tustin','PrewarpFrequency',omega_cruce);

C_discrete= c2d(C,ts,opt)
figure; margin(C); grid on; hold on; bode(C_discrete); legend on;

G_discrete = c2d(G, ts, 'zoh');
L_discrete = C_discrete*G_discrete;
w = logspace(log10(0.1), log10(pi/ts), 1000);

figure; margin(L,w); grid on; hold on; bode(L_discrete,w); legend on;

