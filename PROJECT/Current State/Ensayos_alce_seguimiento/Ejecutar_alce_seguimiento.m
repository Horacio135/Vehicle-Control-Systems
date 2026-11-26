%% Esquivas agresivas con seguimiento y referencia de rapidez constante
% Único caso conservado: Alce PP 02, referencia 20 m/s, transición 10 m.
% Sin sensores, estimador ni ESC.
% Los bloques existentes permanecen intactos. Las referencias son constantes;
% la rapidez real depende de la capacidad física y se registra en las gráficas.
addpath(fileparts(mfilename('fullpath')));
%% Simular
simular_alce_seguimiento;
%% Renderizar el vídeo del alce 02
renderizar_alce_seguimiento;
