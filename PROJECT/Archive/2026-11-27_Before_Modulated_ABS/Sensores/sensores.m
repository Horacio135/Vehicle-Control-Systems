function med = sensores(z, tele, p, k)
% SENSORES Medidas GPS, IMU plana y dirección del vehículo.
% med = sensores(z,tele,p,k). Se conserva la interfaz de cuatro entradas.
% z: estado real del instante medido; utiliza X=z(1), Y=z(2), r=z(6).
% tele.ax, tele.ay: aceleraciones físicas de chasis [m/s^2], no dvx/dt
% ni dvy/dt. En el modelo plano horizontal representan la IMU en el CG,
% alineada con el chasis. No se modelan inclinación, gravedad proyectada
% ni brazo de palanca del montaje de la IMU.
% tele.delta_equivalente: ángulo central virtual real [rad], opcional.
% No es el ángulo del volante. Si falta, med.delta=NaN y
% med.hay_direccion=false; no se inventa una dirección igual a cero.
% p: dt, gps_cada, gps_corte, gps_sigma, acc_sigma, gyro_sigma, gyro_bias.
% Nuevos parámetros opcionales (valores por defecto indicados):
% acc_bias=0, acc_y_sigma=acc_sigma, acc_y_bias=0,
% direccion_sigma=deg2rad(0.1), direccion_bias=0.
% Los sesgos son constantes; ruido gaussiano blanco independiente.
% med.acc se conserva para el EKF P5; med.acc_x es la MISMA muestra.
% med.acc_y añade aceleración lateral; med.delta añade dirección medida.
% IMU y dirección se muestrean cada p.dt; GPS cada gps_cada pasos.
% En ausencia de GPS se devuelven NaN, sin retener ni interpolar medidas.
% El llamador controla rng. El bloque no modifica z, tele ni p.
% Fuerzas, adherencia, estados internos y velocidades de rueda aproximadas
% no se exponen como sensores. Véase LEEME_sensores.txt.

validateattributes(z,{'numeric'},{'real','finite','vector'});
if numel(z)<6
    error('sensores:EstadoIncompleto','z debe incluir al menos X,Y,psi,vx,vy,r.');
end
if ~isstruct(tele) || ~isfield(tele,'ax') || ~isfield(tele,'ay')
    error('sensores:AceleracionesIncompletas', ...
        'tele debe contener ax y ay físicos del intervalo medido.');
end
validateattributes(tele.ax,{'numeric'},{'real','finite','scalar'});
validateattributes(tele.ay,{'numeric'},{'real','finite','scalar'});
validateattributes(k,{'numeric'},{'real','finite','scalar','integer','positive'});
validateattributes(p.dt,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(p.gps_cada,{'numeric'},{'real','finite','scalar','integer','positive'});
validateattributes(p.gps_sigma,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(p.acc_sigma,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(p.gyro_sigma,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(p.gyro_bias,{'numeric'},{'real','finite','scalar'});
if ~isempty(p.gps_corte)
    validateattributes(p.gps_corte,{'numeric'},{'real','finite','vector','numel',2});
    if p.gps_corte(1)>p.gps_corte(2)
        error('sensores:CorteGPSInvalido','El inicio del corte GPS debe preceder al final.');
    end
end

sesgoAx=parametroOpcional(p,'acc_bias',0,false);
ruidoAy=parametroOpcional(p,'acc_y_sigma',p.acc_sigma,true);
sesgoAy=parametroOpcional(p,'acc_y_bias',0,false);
ruidoDireccion=parametroOpcional(p,'direccion_sigma',deg2rad(.1),true);
sesgoDireccion=parametroOpcional(p,'direccion_bias',0,false);

% Canales originales: mismos nombres y mismo orden dentro de cada llamada.
med.acc=tele.ax+sesgoAx+p.acc_sigma*randn;
med.acc_x=med.acc;
med.gyro=z(6)+p.gyro_bias+p.gyro_sigma*randn;
med.t=(k-1)*p.dt;
enCorte=~isempty(p.gps_corte) && med.t>=p.gps_corte(1) && med.t<=p.gps_corte(2);
med.hay_gps=(mod(k-1,p.gps_cada)==0) && ~enCorte;
if med.hay_gps
    med.gps_X=z(1)+p.gps_sigma*randn;
    med.gps_Y=z(2)+p.gps_sigma*randn;
else
    med.gps_X=NaN;med.gps_Y=NaN;
end

% Canales nuevos. El error del lateral es independiente del longitudinal.
med.acc_y=tele.ay+sesgoAy+ruidoAy*randn;
med.hay_direccion=isfield(tele,'delta_equivalente') && ~isempty(tele.delta_equivalente);
if med.hay_direccion
    validateattributes(tele.delta_equivalente,{'numeric'},{'real','finite','scalar'});
    med.delta=tele.delta_equivalente+sesgoDireccion+ruidoDireccion*randn;
else
    med.delta=NaN;
end
end

function valor=parametroOpcional(p,nombre,defecto,noNegativo)
valor=defecto;
if isfield(p,nombre),valor=p.(nombre);end
reglas={'real','finite','scalar'};
if noNegativo,reglas{end+1}='nonnegative';end
validateattributes(valor,{'numeric'},reglas,'sensores',nombre);
end
