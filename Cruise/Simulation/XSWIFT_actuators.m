% ===================================================================
% CONTROL Y GUIADO  
%                                         Departamento de Aeronáutica
%                                              Facultad de Ingeniería
%                                                            U.N.L.P.
% ===================================================================

% anchos de banda de actuadores  
ema.bw_hz = 10; % [hz]
engine.bw =  2; % rad/seg

%%
data.elev.w = ema.bw_hz*2*pi;
data.aler.w = ema.bw_hz*2*pi;
data.rudr.w = ema.bw_hz*2*pi;

engine.A  = -engine.bw;
engine.B  =  engine.bw;
engine.C  =  6492;
