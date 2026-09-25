% Lazo Interno Final

clc; clear; close all;

plotting = false;
% Planta y ganancia propuesta
La = 0.0004;
Ra = 0.066;
G = tf(1, [La, Ra]);
K = 1;

% Lazo abierto y cerrado
Lopen = K * G;
Tcl = feedback(Lopen, 1);   % complementary sensitivity T = L/(1+L)
S = feedback(1, Lopen);    

% Frecuencias para plots
w = logspace(1,5,2000);    % rango rad/s
fc_target = 400; wc_target = 2*pi*fc_target;

% Magnitud/fase de L en el vector w
[magL, phL] = bode(Lopen, w); magL = squeeze(magL); phL = squeeze(phL);

% estimar crossover (primer punto donde mag<=1)
idx = find(magL <= 1, 1, 'first');
if isempty(idx)
    idx = find(abs(magL-1) == min(abs(magL-1)), 1);
end
wc_est = w(idx); fc_est = wc_est/(2*pi);
mag_at_wc = magL(idx);
phase_at_wc = phL(idx);

% Margenes directos
[GM, PM, Wcg, Wcp] = margin(Lopen);
GMdB = 20*log10(GM);
if plotting == true
%% 1) Bode (magnitud en dB y fase) con marca en crossover y anotaciones simples
figure('Name','Bode L(s) - simple','NumberTitle','off');
subplot(2,1,1);
semilogx(w, 20*log10(magL),'LineWidth',1.2); grid on; hold on;
plot(wc_est, 20*log10(mag_at_wc),'ro','MarkerSize',8,'LineWidth',1.4);
xlabel('Frecuencia (rad/s)'); ylabel('Magnitud (dB)');
title('Bode - L(s) (magnitud)'); 
text(wc_est, 20*log10(mag_at_wc)+3, sprintf('wc=%.1f Hz', fc_est), 'HorizontalAlignment','center');

subplot(2,1,2);
semilogx(w, phL,'LineWidth',1.2); grid on; hold on;
plot(wc_est, phase_at_wc,'ro','MarkerSize',8,'LineWidth',1.4);
xlabel('Frecuencia (rad/s)'); ylabel('Fase (deg)');
title('Bode - L(s) (fase)');
text(wc_est, phase_at_wc+8, sprintf('phase@wc=%.1f°', phase_at_wc), 'HorizontalAlignment','center');

% mostrar numericos
fprintf('--- BODE / MARGEN ---\n');
fprintf('wc_est = %.3f rad/s = %.3f Hz\n', wc_est, fc_est);
fprintf('Phase at wc = %.3f deg\n', phase_at_wc);
fprintf('Phase Margin (PM) = %.3f deg\n', PM);
fprintf('Gain Margin (GM) = %.3g (%.2f dB)\n\n', GM, GMdB);

%% 2) Nyquist (usar funcion built-in) - solo plot y punto -1
figure('Name','Nyquist L(s)','NumberTitle','off');
nyquist(Lopen); grid on; hold on;
plot(-1,0,'r+','MarkerSize',10,'LineWidth',2);
title('Nyquist de L(s)  (marcado -1)');

%% 3) Root locus + polos cerrados (K=1) marcado
figure('Name','Root Locus y Polos Cerrados','NumberTitle','off');
rlocus(G); hold on; grid on;
pcl = pole(Tcl);
plot(real(pcl), imag(pcl), 'rx', 'MarkerSize',10, 'LineWidth',2);
for k=1:numel(pcl)
    text(real(pcl(k)), imag(pcl(k))+0.02, sprintf('p%d=%.2f%+.2fj', k, real(pcl(k)), imag(pcl(k))));
end
title('Root locus de G(s) y polos cerrados (K=1)');

% numerico
fprintf('Polos cerrados (K=1):\n'); disp(pcl);

%% 4) Complementary sensitivity T(s) bode y marca en wc_est
[Tmag, Tph] = bode(Tcl, w); Tmag = squeeze(Tmag); Tph = squeeze(Tph);
T_at_wc = interp1(w, Tmag, wc_est);

figure('Name','Complementary sensitivity T(s)','NumberTitle','off');
subplot(2,1,1);
semilogx(w, 20*log10(Tmag),'LineWidth',1.2); grid on; hold on;
plot(wc_est, 20*log10(T_at_wc), 'ro', 'MarkerSize',8,'LineWidth',1.4);
xlabel('Frecuencia (rad/s)'); ylabel('Magnitud T (dB)');
title('Complementary sensitivity T(s) - Magnitud');
text(wc_est, 20*log10(T_at_wc)+3, sprintf('|T(jwc)| = %.2f dB', 20*log10(T_at_wc)), 'HorizontalAlignment','center');

subplot(2,1,2);
semilogx(w, Tph,'LineWidth',1.2); grid on;
xlabel('Frecuencia (rad/s)'); ylabel('Fase (deg)');
title('Complementary sensitivity T(s) - Fase');

fprintf('T(jwc) (linear) = %.6g  => %.3f dB\n\n', T_at_wc, 20*log10(T_at_wc));

[Smag, Sph] = bode(S, w); Smag = squeeze(Smag);
[Ms_val, Ms_idx] = max(Smag);
w_Ms = w(Ms_idx); f_Ms = w_Ms/(2*pi);
figure('Name','Sensitivity S(s)','NumberTitle','off');
subplot(2,1,1);
semilogx(w, 20*log10(Smag),'LineWidth',1.2); grid on; hold on;
xlabel('Frecuencia (rad/s)'); ylabel('Magnitud S (dB)');
title('Sensitivity S(s) - Magnitud');

subplot(2,1,2);
semilogx(w, squeeze(Sph),'LineWidth',1.2); grid on;
xlabel('Frecuencia (rad/s)'); ylabel('Fase (deg)');
title('Sensitivity S(s) - Fase');
end
Kc=3.2;
Tcontrolador=10/2512;
numC = [Tcontrolador,1];
denC = [Kc*Tcontrolador,1];
C = Kc*tf(numC,denC)



save('inner_controller_design.mat', 'La', 'Ra', 'numC', 'denC')