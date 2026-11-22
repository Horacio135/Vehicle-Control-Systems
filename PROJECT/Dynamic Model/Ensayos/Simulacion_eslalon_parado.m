
function [histEslalon0Cin, histEslalon0Din] = Simulacion_eslalon_parado(p, ctrl, fnSimularManiobra, fnMostrarManiobra)

%% SLALOM FROM REST

dtEnsayo = 0.002;

% Test configuration
casoEslalon0.nombre = 'Slalom from rest';
casoEslalon0.T = 24;
casoEslalon0.v0 = 0;
casoEslalon0.vobjetivo = 14;
casoEslalon0.amplitud = deg2rad(14);
casoEslalon0.frecuencia = 0.6;

% Kinematic model
histEslalon0Cin = fnSimularManiobra(p, ctrl, dtEnsayo, casoEslalon0, 2, false);

% Dynamic model
histEslalon0Din = fnSimularManiobra(p, ctrl, dtEnsayo, casoEslalon0, 2, true);

% Comparison plots
fnMostrarManiobra(histEslalon0Cin, histEslalon0Din, p, casoEslalon0.nombre);

end

