function [FxCorregidas, info, mem] = controlABS(FxSolicitadas, z, p, mem, dt)
% ABS por capacidad con descarga, mantenimiento y recarga independientes.
% Orden: [DI DD TI TD]. Solo modifica las demandas de frenada negativas.
% Interfaz con memoria explícita: [f,info,mem]=controlABS(f,z,p,mem,dt).
% El llamador debe conservar mem; [] inicia una ejecución independiente.
% Límite normal 0.98*Fmax; descarga hasta 0.80*Fmax en 30 ms,
% mantiene 10 ms y recupera hasta 0.98 en 60 ms. Ajustables con p.abs.
% Son parámetros iniciales de una aproximación por fuerza, NO una calibración
% identificada de Formula Student/Brembo ni regulación de slip real.
% Fmax se calcula igual que en el bloque anterior. No descuenta Fy.
% El perfil es continuo en su factor normalizado; cambios de Fmax o de la
% demanda pueden cambiar directamente la fuerza. No es un modelo hidráulico.
% Soltar el freno o pedir tracción reinicia el canal y no deja frenada residual.
% Fases: 0 normal; 1 descarga; 2 mantenimiento; 3 recarga.

validateattributes(FxSolicitadas,{'numeric'},{'real','finite','vector','numel',4});
forma=size(FxSolicitadas);solicitadas=double(FxSolicitadas(:).');corregidas=solicitadas;
frenada=solicitadas<0;
par=parametrosABS(p);
if nargin<4 || isempty(mem)
    mem=struct('factor',par.factorActivacion*ones(1,4),'fase',zeros(1,4),'tiempoFase',zeros(1,4));
else
    assert(isstruct(mem)&&all(isfield(mem,{'factor','fase','tiempoFase'})), ...
        'controlABS:MemoriaInvalida','La memoria ABS debe contener factor, fase y tiempoFase.');
    validateattributes(mem.factor,{'numeric'},{'real','finite','size',[1 4]});
    validateattributes(mem.fase,{'numeric'},{'real','finite','size',[1 4],'integer','>=',0,'<=',3});
    validateattributes(mem.tiempoFase,{'numeric'},{'real','finite','size',[1 4],'nonnegative'});
end
% No modificar órdenes positivas; cada canal sin frenada pierde su ciclo.
mem.factor(~frenada)=par.factorActivacion;mem.fase(~frenada)=0;mem.tiempoFase(~frenada)=0;
info=struct('activo',any(frenada),'limitadas',false(forma),'modulando',false(forma), ...
    'disparadas',false(forma),'Fzw',nan(forma),'Fmax_peak_w',nan(forma), ...
    'eta_brake',nan(forma),'Fmax_w',nan(forma),'limiteNormal',nan(forma), ...
    'factorAplicado',reshape(mem.factor,forma),'fase',reshape(mem.fase,forma),'parametros',par);
if ~any(frenada),FxCorregidas=reshape(corregidas,forma);return;end
if nargin<4 || nargout<3
    error('controlABS:MemoriaNecesaria', ...
        'Para frenar use [f,info,mem]=controlABS(f,z,p,mem,dt) y conserve mem entre pasos.');
end
if nargin<5
    assert(isfield(p,'dt'),'controlABS:PasoNecesario','Indique dt o defina p.dt.');dt=p.dt;
end
validateattributes(dt,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(z,{'numeric'},{'real','finite','vector','numel',20});z=z(:);
% Mismas expresiones de cargas y capacidad que el ABS anterior.
ax_anterior=z(7);ay_anterior=z(8);
dFzx=p.m*ax_anterior*p.hcg/p.L;
dFzyd=p.kRoll*p.m*ay_anterior*p.hcg/p.td;
dFzyt=(1-p.kRoll)*p.m*ay_anterior*p.hcg/p.tt;
Fzw=p.Fzw0+[-dFzx/2,-dFzx/2,dFzx/2,dFzx/2]+[-dFzyd,dFzyd,-dFzyt,dFzyt];
if any(~isfinite(Fzw))||any(Fzw<0)
    error('controlABS:CargaVerticalInvalida','Cargas verticales inválidas: el ABS no modifica el modelo físico.');
end
Fmax_peak_w=p.muw.*Fzw;
eta_brake=max(p.brake_slide_ratio,min(1,z(13:16).'));
Fmax_w=eta_brake.*Fmax_peak_w;
if any(~isfinite(Fmax_w))||any(Fmax_w<0)
    error('controlABS:CapacidadInvalida','La capacidad de adherencia debe ser finita y no negativa.');
end
disparadas=false(1,4);modulando=false(1,4);
for rueda=find(frenada)
    if Fmax_w(rueda)<=eps
        corregidas(rueda)=0;mem.factor(rueda)=par.factorActivacion;mem.fase(rueda)=0;mem.tiempoFase(rueda)=0;
        continue
    end
    alta=abs(solicitadas(rueda))>=par.factorActivacion*Fmax_w(rueda);
    if mem.fase(rueda)==0
        mem.factor(rueda)=par.factorActivacion;
        if alta
            % Se alcanza el techo en este instante. La descarga comienza
            % durante el siguiente intervalo, sin un salto hasta el mínimo.
            mem.fase(rueda)=1;mem.tiempoFase(rueda)=0;disparadas(rueda)=true;
        end
    else
        [mem.factor(rueda),mem.fase(rueda),mem.tiempoFase(rueda)]= ...
            avanzarFases(mem.fase(rueda),mem.tiempoFase(rueda),dt,alta,par);
    end
    modulando(rueda)=mem.fase(rueda)~=0;
    corregidas(rueda)=-min(abs(solicitadas(rueda)),mem.factor(rueda)*Fmax_w(rueda));
end
FxCorregidas=reshape(corregidas,forma);
info.limitadas=reshape(frenada & abs(corregidas)<abs(solicitadas)-1e-10,forma);
info.modulando=reshape(modulando,forma);info.disparadas=reshape(disparadas,forma);
info.Fzw=reshape(Fzw,forma);info.Fmax_peak_w=reshape(Fmax_peak_w,forma);
info.eta_brake=reshape(eta_brake,forma);info.Fmax_w=reshape(Fmax_w,forma);
info.limiteNormal=reshape(par.factorActivacion*Fmax_w,forma);
info.factorAplicado=reshape(mem.factor,forma);info.fase=reshape(mem.fase,forma);
end

function par=parametrosABS(p)
par=struct('factorActivacion',.98,'factorDescarga',.80, ...
    'tiempoDescarga',.030,'tiempoMantenimiento',.010,'tiempoRecarga',.060);
if isfield(p,'abs')
    assert(isstruct(p.abs),'controlABS:ParametrosInvalidos','p.abs debe ser una estructura.');
    for nombre=fieldnames(par)'
        if isfield(p.abs,nombre{1}),par.(nombre{1})=p.abs.(nombre{1});end
    end
end
validateattributes(par.factorActivacion,{'numeric'},{'real','finite','scalar','positive','<=',1});
validateattributes(par.factorDescarga,{'numeric'},{'real','finite','scalar','nonnegative','<',par.factorActivacion});
validateattributes(par.tiempoDescarga,{'numeric'},{'real','finite','scalar','positive'});
validateattributes(par.tiempoMantenimiento,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(par.tiempoRecarga,{'numeric'},{'real','finite','scalar','positive'});
end

function [factor,fase,tiempo]=avanzarFases(fase,tiempo,dt,alta,par)
% Consumir el intervalo completo, también si dt cruza una frontera de fase.
restante=dt;amplitud=par.factorActivacion-par.factorDescarga;factor=par.factorActivacion;
while restante>1e-12
    if fase==0
        if ~alta,factor=par.factorActivacion;break;end
        fase=1;tiempo=0;
    end
    duraciones=[par.tiempoDescarga par.tiempoMantenimiento par.tiempoRecarga];
    duracion=duraciones(fase);
    paso=min(restante,max(0,duracion-tiempo));tiempo=tiempo+paso;restante=restante-paso;
    switch fase
        case 1,factor=par.factorActivacion-amplitud*min(1,tiempo/duracion);
        case 2,factor=par.factorDescarga;
        case 3,factor=par.factorDescarga+amplitud*min(1,tiempo/duracion);
    end
    if tiempo>=duracion-1e-12
        fase=fase+1;tiempo=0;
        if fase==4
            factor=par.factorActivacion;fase=0;
            if alta,fase=1;end
        end
    end
end
factor=max(par.factorDescarga,min(par.factorActivacion,factor));
end
