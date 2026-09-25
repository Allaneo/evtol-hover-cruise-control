% ===================================================================
% CONTROL Y GUIADO  
%                                         Departamento de Aeronáutica
%                                              Facultad de Ingeniería
%                                                            U.N.L.P.
% ===================================================================
% 
% _________________________________________________________________

deg2rad = pi/180;
I3 = [1 1 1];

% alfa, beta, TAS, height
sens.adc.noise.gain  = [[1 1]*0.01*deg2rad ... % ruido en alfa y beta
                        0.02 ...               % ruido en TAS
                        0.2];                  % ruido en altura
sens.adc.noise.seeds = round(rand(size(sens.adc.noise.gain)) * 100000);

% euler, pqr, sfrc
sens.ins.noise.gain  = [[I3*0.0000 ...          % ruido en la medición de ángulos de euler
                        I3*sqrt(0.0033)] * deg2rad ... % ruido en la medición de velocidad angular
                        I3*sqrt(0.0012)];              % ruido en la medición de aceleración
sens.ins.noise.seeds = round(rand(size(sens.ins.noise.gain)) * 100000);

if disable_measurement_noise
    sens.adc.noise.gain = zeros(size(sens.adc.noise.gain));
    sens.ins.noise.gain = zeros(size(sens.ins.noise.gain));
end


