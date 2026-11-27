function [FxFinales, deltaFinal, mem, info] = controlTotal(est, ref, trayectoria, obstaculo, p, mem, dt)
% Coordinación de los bloques de control, sin modificar la dinámica.
% Entradas:
%   est: salida del estimador (X,Y,psi,vx) y est.zControl, estado estimado
%        de 20 componentes compatible con ABS y TC. La plantilla lo forma
%        con el adaptador y la capacidad predicha, sin leer memorias reales.
%   ref.v_set: velocidad solicitada [m/s]; trayectoria: puntos Nx2 [m].
%   obstaculo: estructura X,Y; [] indica que no hay un obstáculo detectado.
%   p: parámetros de los bloques; mem: memorias de la llamada anterior,
%      o [] en la primera llamada; dt: paso de simulación [s].
% Salidas:
%   FxFinales: fuerzas por rueda [DI DD TI TD], positivas en tracción [N].
%   deltaFinal: ángulo de dirección de las ruedas delanteras [rad].
%   mem: memorias actualizadas; info: modo y diagnósticos.
% Prioridad: emergencia > ABS o TC. ABS y TC nunca se llaman a la vez.
% Emergencia evita ambos limitadores y mantiene el enclavamiento del bloque
% existente. Para reiniciarlo se debe reiniciar mem.emergencia.
% Tracción igual en ambas traseras: se usa el menor límite de las dos.
% Frenada nominal [.3 .3 .2 .2], corregida individualmente por el ABS.
% La plantilla conecta u=[FxFinales(:);deltaFinal] con la dinámica vigente.
% mem.abs conserva las fases de modulación entre llamadas de frenada.

if nargin<6 || isempty(mem), mem=struct(); end
if ~isfield(mem,'lateral'), mem.lateral=[]; end
if ~isfield(mem,'longitudinal'), mem.longitudinal=[]; end
if ~isfield(mem,'abs'), mem.abs=[]; end
if ~isfield(mem,'emergencia'), mem.emergencia=struct('latched',false); end
validateattributes(dt,{'numeric'},{'real','finite','scalar','positive'});

[deltaSolicitado,mem.lateral]=controlLateral(est,trayectoria,p,mem.lateral);
[FxSolicitada,mem.longitudinal]=controlLongitudinal(est,ref,p,mem.longitudinal,dt);

% Sin detección nueva, conservar una emergencia previamente enclavada.
if isempty(obstaculo)
    emergenciaActiva=mem.emergencia.latched;
    FxEmergencia=-p.Fx_brake_max;
    deltaEmergencia=deltaSolicitado;
else
    [FxEmergencia,deltaEmergencia,emergenciaActiva,mem.emergencia]= ...
        controlEmergencia(est,obstaculo,deltaSolicitado,p,mem.emergencia);
end

info=struct('modo','libre','emergenciaActiva',emergenciaActiva, ...
    'ABSActivo',false,'TCActivo',false,'ABS',[],'TC',[], ...
    'FxSolicitada',FxSolicitada,'deltaSolicitado',deltaSolicitado);
deltaFinal=deltaSolicitado;
if emergenciaActiva
    % La orden de emergencia es la única que llega a la salida.
    FxFinales=repartirFuerza(FxEmergencia);
    deltaFinal=deltaEmergencia;
    mem.abs=[]; % La emergencia tiene prioridad; no reanudar un ciclo antiguo.
    info.modo='emergencia';
    return
end

FxFinales=repartirFuerza(FxSolicitada);
if FxSolicitada==0, mem.abs=[]; return; end
if ~isfield(est,'zControl')
    error('controlTotal:EstadoEstimadoIncompleto', ...
        ['ABS y TC necesitan est.zControl (20 componentes). El estimador ' ...
         'actual no proporciona las cargas y memorias de adherencia necesarias.']);
end
if FxSolicitada<0
    habilitarABS=true;
    if isfield(p,'abs') && isfield(p.abs,'habilitado')
        validateattributes(p.abs.habilitado,{'logical','numeric'},{'real','finite','scalar'});
        assert(p.abs.habilitado==0 || p.abs.habilitado==1,'controlTotal:InterruptorABS','p.abs.habilitado debe ser lógico.');
        habilitarABS=logical(p.abs.habilitado);
    end
    if habilitarABS
        [FxFinales,info.ABS,mem.abs]=controlABS(FxFinales,est.zControl,p,mem.abs,dt);
        info.ABSActivo=true;
        info.modo='frenada';
    else
        mem.abs=[];
        info.modo='frenada_sin_ABS';
    end
else
    mem.abs=[]; % La tracción cancela la memoria del ciclo de frenada.
    [FxFinales(3:4),info.TC]=controlTraccion(FxFinales(3:4),est.zControl,p,dt);
    % El motor entrega la misma fuerza a ambas ruedas motrices.
    % La rueda de menor capacidad determina la reducción común de potencia.
    FxFinales(3:4)=min(FxFinales(3:4));
    info.TCActivo=true;
    info.modo='traccion';
end
end

function fuerzas=repartirFuerza(fuerzaTotal)
if fuerzaTotal<0
    fuerzas=fuerzaTotal*[.3 .3 .2 .2];
else
    fuerzas=fuerzaTotal*[0 0 .5 .5];
end
end
