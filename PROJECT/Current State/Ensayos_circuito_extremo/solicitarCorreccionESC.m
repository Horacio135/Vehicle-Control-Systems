function [freno,mem,info]=solicitarCorreccionESC(est,delta,p,mem,dt)
% Ley ESC V1: separa la petición de freno del actuador ABS final.
% Mismas ecuaciones, umbrales, selección y rampas que controlESC original.
% est: vx,vy,r,ax,ay. Estados conocidos en este ensayo sin sensores/EKF.
% La tracción se retira mientras hay intervención o descarga del freno.
if nargin<4||isempty(mem),mem=struct('rRef',0,'activo',false,'freno',zeros(1,4),'abs',[]);end
c=p.esc;v=max(est.vx,c.velocidadMinima);
Ku=p.m/p.L*(p.lt/p.Cad-p.ld/p.Cat);
referencia=est.vx*delta/(p.L+Ku*est.vx^2);
limite=c.factorAdherencia*min(p.muw)*p.g/v;
referencia=max(-limite,min(limite,referencia));
mem.rRef=mem.rRef+(1-exp(-dt/c.tauReferencia))*(referencia-mem.rRef);
beta=atan2(est.vy,max(est.vx,.1));errorR=mem.rRef-est.r;
if est.vx<c.velocidadMinima
 mem.activo=false;
elseif ~mem.activo
 mem.activo=abs(errorR)>c.errorActivar || abs(beta)>c.betaActivar;
elseif abs(errorR)<c.errorSoltar && abs(beta)<c.betaSoltar
 mem.activo=false;
end
momento=0;objetivo=zeros(1,4);rueda=0;
if mem.activo
 % beta positiva exige, con estos ejes, aumentar r para reducir su crecimiento.
 momento=c.Kr*errorR+c.Kbeta*beta;
 momento=max(-c.momentoMaximo,min(c.momentoMaximo,momento));
 % Sobreviraje/deriva: delantera. Subviraje: trasera interior.
 sobre=est.r*mem.rRef>0 && abs(est.r)>abs(mem.rRef);
 if sobre || abs(beta)>c.betaActivar
  candidatas=1:2;
 else
  candidatas=3:4;
 end
 dw=angulosAckermann(delta,p);
 palanca=p.yw.*cos(dw)-p.xw.*sin(dw);
 validas=candidatas(palanca(candidatas)*momento>0);
 if ~isempty(validas)
  [~,j]=max(abs(palanca(validas)));rueda=validas(j);
  objetivo(rueda)=min(c.frenoMaximo,abs(momento/palanca(rueda)));
 end
end
% Aplicación y descarga continuas. No trasladar de golpe la fuerza entre ruedas.
tau=c.tauAplicacion*ones(1,4);tau(objetivo<mem.freno)=c.tauDescarga;
mem.freno=mem.freno+(1-exp(-dt./tau)).*(objetivo-mem.freno);
mem.freno(mem.freno<.1)=0;
interviene=any(mem.freno>0);
freno=mem.freno;
info=struct('activo',interviene,'rRef',mem.rRef,'beta',beta,'errorR',errorR, ...
 'momentoSolicitado',momento,'rueda',rueda,'frenoPrevioABS',mem.freno);
end
