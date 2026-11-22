
function [histEslalon0Cin, histEslalon0Din] = Simulacion_eslalon_parado(p, ctrl, fnSimularManiobra, fnMostrarManiobra)

%% ESLALON DESDE PARADO

dtEnsayo = 0.002;

% Configuracion del ensayo
casoEslalon0.nombre = 'Eslalon desde parado';
casoEslalon0.T = 24;
casoEslalon0.v0 = 0;
casoEslalon0.vobjetivo = 14;
casoEslalon0.amplitud = deg2rad(14);
casoEslalon0.frecuencia = 0.6;

% Modelo cinematico
histEslalon0Cin = fnSimularManiobra(p, ctrl, dtEnsayo, casoEslalon0, 2, false);

% Modelo dinamico
histEslalon0Din = fnSimularManiobra(p, ctrl, dtEnsayo, casoEslalon0, 2, true);

% Graficas comparativas
fnMostrarManiobra(histEslalon0Cin, histEslalon0Din, p, casoEslalon0.nombre);

end

