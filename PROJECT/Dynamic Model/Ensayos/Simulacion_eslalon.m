function [histEslalonCin, histEslalonDin] = Simulacion_eslalon(p, ctrl, fnSimularManiobra, fnMostrarManiobra)

dtEnsayo = 0.002;
caso.nombre = 'Slalom';
caso.T = 24;
caso.v0 = 14;
caso.amplitud = deg2rad(14);
caso.frecuencia = 0.6; % Hz
j = 4;
cin = fnSimularManiobra(p, ctrl, dtEnsayo, caso, j, false);
din = fnSimularManiobra(p, ctrl, dtEnsayo, caso, j, true);

fnMostrarManiobra(cin, din, p, caso.nombre);

histEslalonCin = cin;
histEslalonDin = din;

end