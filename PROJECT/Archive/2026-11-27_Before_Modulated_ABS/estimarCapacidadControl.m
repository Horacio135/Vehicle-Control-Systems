function [capacidad,mem]=estimarCapacidadControl(est,uAnterior,p,mem,dt)
% Predicción para ABS/TC. Solo recibe estimaciones, órdenes conocidas y p.
% No estima mu a partir de datos ni observa las memorias verdaderas de planta.
% Emplea mu nominal y un modelo interno de recuperación/bloqueo del frenado.
% mem contiene únicamente su propia eta_brake y su propio modo de bloqueo.
% Una proyección de cargas no negativas evita interpretar ruido como contacto
% físico negativo; capacidad.cargas_proyectadas hace visible esa intervención.
if nargin<4 || isempty(mem),mem=struct('eta_brake',ones(1,4),'bloqueo',false(1,4));end
validateattributes(dt,{'numeric'},{'real','finite','scalar','nonnegative'});
validateattributes(uAnterior,{'numeric'},{'real','finite','vector','numel',5});
validateattributes([est.ax est.ay est.vx est.vy est.r],{'numeric'},{'real','finite','numel',5});
dX=p.m*est.ax*p.hcg/p.L;
dD=p.kRoll*p.m*est.ay*p.hcg/p.td;
dT=(1-p.kRoll)*p.m*est.ay*p.hcg/p.tt;
variacion=[-dX/2 -dX/2 dX/2 dX/2]+[-dD dD -dT dT];
cargas=p.Fzw0+variacion;
proyectadas=any(cargas<0);
escala=1;
if proyectadas
    negativas=variacion<0;
    escala=min([1 p.Fzw0(negativas)./(-variacion(negativas))])*(1-1e-12);
    cargas=p.Fzw0+escala*variacion;
end
pico=p.muw.*cargas;
fuerzas=uAnterior(1:4).';
if sum(fuerzas)>=0
    % El modelo elegido reinicia esas memorias en tracción/rodadura.
    mem.eta_brake=ones(1,4);mem.bloqueo=false(1,4);
else
    demanda=max(0,-fuerzas);ratio=demanda./max(pico,eps);
    mem.bloqueo(ratio>p.brake_lock_on)=true;
    mem.bloqueo(ratio<p.brake_lock_off)=false;
    suelto=demanda==0;mem.bloqueo(suelto)=false;
    delta=uAnterior(5); % Orden conocida de respaldo si falta medida.
    if isfield(est,'hay_direccion')&&est.hay_direccion&&isfinite(est.delta),delta=est.delta;end
    dw=angulosAckermann(delta,p);
    vx=est.vx-est.r*p.yw;vy=est.vy+est.r*p.xw;
    vxLocal=vx.*cos(dw)+vy.*sin(dw);
    objetivo=ones(1,4);
    perdida=p.brake_slide_ratio+(1-p.brake_slide_ratio)*exp(-abs(vxLocal)/p.v_brake_slide);
    objetivo(mem.bloqueo)=perdida(mem.bloqueo);
    tau=p.tau_brake_rec*ones(1,4);tau(mem.bloqueo)=p.tau_brake_loss;tau(suelto)=p.tau_brake_release;
    mem.eta_brake=mem.eta_brake+(1-exp(-dt./tau)).*(objetivo-mem.eta_brake);
    mem.eta_brake=max(p.brake_slide_ratio,min(1,mem.eta_brake));
end
capacidad=struct('Fzw',cargas,'Fmax_peak_w',pico,'eta_brake',mem.eta_brake, ...
    'Fmax_w',pico.*mem.eta_brake,'bloqueo_predicho',mem.bloqueo, ...
    'ax_cargas',escala*est.ax,'ay_cargas',escala*est.ay, ...
    'cargas_proyectadas',proyectadas,'origen','Modelo interno con señales estimadas');
end
