function [f,mem,info]=coordinarAyudasExtremo(est,nominal,delta,p,ayudas,mem,dt)
% Peticiones ESC y conductor se suman ANTES de una única actuación del ABS.
if isempty(mem),mem=struct('ABS',[],'ESC',[]);end
if nominal>=0,f=nominal*p.drive_w;else,f=nominal*p.brake_w;end
info=struct('TC',false,'ABS',false,'ESC',false,'rRef',NaN,'beta',atan2(est.vy,est.vx), ...
 'momento',0,'seleccion',0,'frenoESC',zeros(1,4),'factorABS',ones(1,4));
if ayudas.TC && nominal>0
 [f(3:4),it]=controlTraccion(f(3:4),est.zControl,p,dt);f(3:4)=min(f(3:4));
 info.TC=any(it.limitadas);
end
if ayudas.ESC
 [freno,mem.ESC,ie]=solicitarCorreccionESC(est,delta,p,mem.ESC,dt);
 info.ESC=ie.activo;info.rRef=ie.rRef;info.momento=ie.momentoSolicitado;
 info.seleccion=ie.rueda;info.frenoESC=freno;
 if ie.activo,f=min(f,0)-freno;end
end
if ayudas.ABS && any(f<0)
 [f,ia,mem.ABS]=controlABS(f,est.zControl,p,mem.ABS,dt);
 info.ABS=any(ia.limitadas);info.factorABS=ia.factorAplicado;
else
 mem.ABS=[];
end
assert(~(any(f>0)&&any(f<0)));
end
