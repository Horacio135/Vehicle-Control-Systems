function [est, mem] = estimadorEstado(med, uPrevious, p, mem, dt) %#ok<INUSD>
% ESTIMADORESTADO EKF P5 ampliado a movimiento lateral y dirección medida.
% Misma firma y ciclo de predicción IMU / corrección GPS. No recibe z ni tele.
% med: acc_x (o acc), acc_y, gyro, hay_gps, gps_X, gps_Y;
%      delta y hay_direccion opcionales. Un dato angular ausente no es cero.
% mem.xh=[X;Y;psi;vx;vy;ay;delta], mem.P de 7x7.
% Admite memorias anteriores de 4 o 5 estados: añade vy si falta, ay y delta
% con incertidumbre. No extrae esos estados del vehículo real.
% p.Q: covarianza por paso de referencia para [X Y psi vx vy], de 5x5;
% admite Q antigua 4x4 añadiendo incertidumbre lateral. p.R: GPS 2x2.
% p.estimador.q_ay y q_direccion: intensidades de paseo aleatorio, por segundo.
% ay y delta reciben correcciones Kalman con sus respectivas medidas.
% No se impone r=vx*tan(delta)/L: esa igualdad falla durante la deriva.
% dt=0 permite asimilar la primera medida sin adelantar la simulación.
% est.acc_x/ax y est.r conservan las medidas longitudinal y de giro de P5:
% esta ampliación NO estima sesgos ni añade un nuevo filtro de giróscopo.
% uPrevious conserva la interfaz; no sustituye la dirección por la orden real.

validateattributes(dt,{'numeric'},{'real','finite','scalar','nonnegative'});
if isfield(med,'acc_x'),ax=med.acc_x;else,ax=med.acc;end
ay=med.acc_y;r=med.gyro;
validateattributes([ax ay r],{'numeric'},{'real','finite','vector','numel',3});
validateattributes(mem.xh,{'numeric'},{'real','finite','vector'});
x=mem.xh(:);n=numel(x);
if ~ismember(n,[4 5 7])
    error('estimadorEstado:EstadoInvalido','La memoria debe tener 4, 5 o 7 componentes.');
end
validateattributes(mem.P,{'numeric'},{'real','finite','size',[n n]});
P=mem.P;
if n==4,x=[x;0];P=blkdiag(P,1);end
if numel(x)==5,x=[x;0;0];P=blkdiag(P,25,pi^2);end
if ~isfield(mem,'direccionInicializada'),mem.direccionInicializada=false;end

qAy=25;qDireccion=.002;
if isfield(p,'estimador')
    if isfield(p.estimador,'q_ay'),qAy=p.estimador.q_ay;end
    if isfield(p.estimador,'q_direccion'),qDireccion=p.estimador.q_direccion;end
end
validateattributes([qAy qDireccion],{'numeric'},{'real','finite','nonnegative'});
sigmaAy=p.acc_sigma;
if isfield(p,'acc_y_sigma'),sigmaAy=p.acc_y_sigma;end
sigmaDireccion=deg2rad(.1);
if isfield(p,'direccion_sigma'),sigmaDireccion=p.direccion_sigma;end
validateattributes([sigmaAy sigmaDireccion p.acc_sigma p.gyro_sigma], ...
    {'numeric'},{'real','finite','nonnegative'});

% Nuevas magnitudes: evolución aleatoria y corrección de la medida actual.
% La aceleración lateral representa el intervalo que se va a integrar.
P(6,6)=P(6,6)+qAy*dt;P(7,7)=P(7,7)+qDireccion*dt;
H=zeros(1,7);H(6)=1;
[x,P]=corregirMedida(x,P,ay,H,sigmaAy^2);
direccionDisponible=isfield(med,'delta') && isscalar(med.delta) && isfinite(med.delta);
if isfield(med,'hay_direccion'),direccionDisponible=direccionDisponible && logical(med.hay_direccion);end
if direccionDisponible
    H=zeros(1,7);H(7)=1;
    [x,P]=corregirMedida(x,P,med.delta,H,sigmaDireccion^2);
    mem.direccionInicializada=true;
end

% Predicción de movimiento: misma integración Euler de P5, ampliada a vy.
psi=x(3);vx=x(4);vy=x(5);c=cos(psi);s=sin(psi);
xPredicha=x;
xPredicha(1)=x(1)+(vx*c-vy*s)*dt;
xPredicha(2)=x(2)+(vx*s+vy*c)*dt;
xPredicha(3)=psi+r*dt;
xPredicha(4)=vx+(ax+r*vy)*dt;
xPredicha(5)=vy+(x(6)-r*vx)*dt;
F=eye(7);
F(1,3)=(-vx*s-vy*c)*dt;F(1,4)=c*dt;F(1,5)=-s*dt;
F(2,3)=( vx*c-vy*s)*dt;F(2,4)=s*dt;F(2,5)= c*dt;
F(4,5)=r*dt;F(5,4)=-r*dt;F(5,6)=dt;
Q=p.Q;
if isequal(size(Q),[4 4]),Q=blkdiag(Q,.1^2);end
validateattributes(Q,{'numeric'},{'real','finite','size',[5 5]});
pasoReferencia=dt;
if isfield(p,'dt'),pasoReferencia=p.dt;end
if pasoReferencia==0,pasoReferencia=1;end
Qcompleta=zeros(7);Qcompleta(1:5,1:5)=Q*(dt/pasoReferencia);
% Propagar también la incertidumbre del acelerómetro x y del giróscopo.
G=zeros(7,2);G(4,1)=dt;G(3,2)=dt;G(4,2)=vy*dt;G(5,2)=-vx*dt;
Ppredicha=F*P*F.'+Qcompleta+G*diag([p.acc_sigma^2 p.gyro_sigma^2])*G.';

% Corrección GPS original, con columnas adicionales para el estado ampliado.
if med.hay_gps
    gps=[med.gps_X;med.gps_Y];
    validateattributes(gps,{'numeric'},{'real','finite','vector','numel',2});
    H=zeros(2,7);H(1,1)=1;H(2,2)=1;
    [x,P]=corregirMedida(xPredicha,Ppredicha,gps,H,p.R);
else
    x=xPredicha;P=Ppredicha;
end
mem.xh=x;mem.P=(P+P.')/2;
est=struct('X',x(1),'Y',x(2),'psi',x(3),'vx',x(4),'vy',x(5), ...
    'r',r,'ax',ax,'acc_x',ax,'ay',x(6),'acc_y',x(6),'delta',NaN, ...
    'hay_direccion',mem.direccionInicializada,'direccion_actualizada',direccionDisponible);
if mem.direccionInicializada,est.delta=x(7);end
end

function [x,P]=corregirMedida(x,P,medida,H,R)
% Misma corrección Kalman; forma Joseph para preservar la covarianza.
S=H*P*H.'+R;
if all(S==0,'all')
    if any(abs(medida-H*x)>1e-10)
        error('estimadorEstado:IncertidumbreNula','Medida incompatible con una incertidumbre nula.');
    end
    return
end
if rcond(S)<1e-14
    error('estimadorEstado:CovarianzaSingular','Covarianza de innovación singular.');
end
K=P*H.'/S;
x=x+K*(medida-H*x);
I=eye(numel(x));P=(I-K*H)*P*(I-K*H).'+K*R*K.';
P=(P+P.')/2;
end
