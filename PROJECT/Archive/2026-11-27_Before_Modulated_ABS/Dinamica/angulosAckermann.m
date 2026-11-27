function delta_w=angulosAckermann(delta,p)
% Ackermann ideal, orden [DI DD TI TD]. Ángulos de rueda en radianes.
% delta es el ángulo virtual delantero del modelo bicicleta, no del volante.
% Convenciones del proyecto: y positivo a la izquierda, delta positivo a
% izquierdas. La rueda interior gira más que la exterior. Traseras sin giro.
% La construcción usa un centro común sobre la prolongación del eje trasero:
% delta_i=atan2(L*sin(delta),L*cos(delta)-y_i*sin(delta)).
% Ackermann impone geometría de dirección, no alpha=0 durante la dinámica.
% p.delta_max conserva su significado de límite del ángulo virtual central;
% el ángulo real de la rueda interior puede ser mayor que ese valor.
validateattributes(delta,{'numeric'},{'real','finite','scalar'});
L=p.ld+p.lt;
validateattributes(L,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(p.yw,{'numeric'},{'real','finite','vector','numel',4});
if abs(delta)>=pi/2
    error('Ackermann:AnguloInvalido','El ángulo equivalente debe estar entre -pi/2 y pi/2.');
end
if isfield(p,'direccion')&&isfield(p.direccion,'ackermann')&&~p.direccion.ackermann
    delta_w=[delta delta 0 0];return
end
if delta==0,delta_w=zeros(1,4);return;end
denominador=L*cos(delta)-p.yw(1:2).*sin(delta);
if any(denominador<=0)
    error('Ackermann:RadioInvalido','El giro solicitado sitúa el centro de giro dentro de la vía delantera.');
end
delta_w=[atan2(L*sin(delta),denominador) 0 0];
end
