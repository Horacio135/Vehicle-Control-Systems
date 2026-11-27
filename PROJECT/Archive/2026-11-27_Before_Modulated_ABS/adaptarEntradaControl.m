function zControl=adaptarEntradaControl(est,capacidad)
% Adaptación EXTERNA al contrato numérico de los controladores existentes.
% No recibe el estado z ni la telemetría real de la planta.
% Solo 7:8 (aceleraciones para cargas) y 13:16 (memoria propia de freno)
% son leídos por ABS/TC. El resto mantiene la forma de la interfaz antigua.
% 9:12 son posiciones reservadas a 1; NO se estima eta_slip de la planta.
% 17:20 son indicadores PREDICHOS por el modelo del control, no sensores.
% El vector es una adaptación de interfaz, no un estado físico reconstruido.
zControl=[est.X;est.Y;est.psi;est.vx;est.vy;est.r; ...
    capacidad.ax_cargas;capacidad.ay_cargas;ones(4,1); ...
    capacidad.eta_brake(:);double(capacidad.bloqueo_predicho(:))];
validateattributes(zControl,{'numeric'},{'real','finite','vector','numel',20});
end
