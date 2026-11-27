function [p,cfg,plan]=configuracionIntegracion
% Configuración de integración: conserva los parámetros de la versión activa.
[p,cfg,plan]=configuracionSoloTraccion;
% Sensores y EKF P5: últimos parámetros definidos en la plantilla modular.
p.gps_sigma=.5;p.acc_sigma=.2;p.gyro_sigma=.01;
p.gps_cada=25;p.gps_corte=[];p.gyro_bias=0;
% Ampliación de sensores 26/11/2026. Hipótesis iniciales no identificadas.
p.acc_bias=0;                     % Sesgo longitudinal [m/s^2].
p.acc_y_sigma=.2;p.acc_y_bias=0;  % Ruido y sesgo lateral [m/s^2].
p.direccion_sigma=deg2rad(.1);     % Desviación típica angular [rad].
p.direccion_bias=0;               % Sesgo angular [rad].
% med.acc_x replica med.acc; med.acc_y y med.delta son nuevos canales.
% El EKF usa ay y dirección; ABS/TC mantienen su propia predicción interna.
% EKF: [X Y psi vx vy]; Q reducida reactivada para futuras pruebas.
% 27/11/2026: se recupera la configuración de la vuelta de 47.370 s.
% Sensores, R, P inicial y algoritmo permanecen sin cambios.
p.Q=diag([.001 .001 .00001 .003 .003].^2);
p.R=diag([p.gps_sigma p.gps_sigma].^2);
% Intensidad por segundo de cambios en ay y dirección (hipótesis iniciales).
p.estimador=struct('q_ay',25,'q_direccion',.002);
cfg.usarSensoresEstimador=true;
% Capacidad de ABS/TC predicha a partir de estimaciones y órdenes anteriores.
% No se rellena un estado físico con memorias reales de la planta.
cfg.compararEstadoPerfecto=true;
% Destino común de futuras simulaciones y renderizaciones del lazo completo.
carpetaEstadoActual=fileparts(fileparts(mfilename('fullpath')));
cfg.carpetaResultados=fullfile(carpetaEstadoActual,'Resultados');
cfg.guardarResultados=true;
end


