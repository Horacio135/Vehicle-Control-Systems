%% Lazo completo de 33 curvas: todos los controles reciben estimaciones
base=fileparts(mfilename('fullpath'));raiz=fileparts(fileparts(base));addpath(raiz,fileparts(base),'-begin');addpath(base,'-begin');
for sub={'Control','Dinamica','Configuracion','Sensores','Estimacion'},addpath(fullfile(raiz,sub{1}),'-begin');end
[p,~,plan]=configuracionIntegracion;p=configuracionESC(p);
plan.ayMax=11;plan.axAcel=6;plan.axFreno=12;
pista=generarCircuitoExtremo;perfil=planificarExtremo(pista,40,plan);
salida=fullfile(raiz,'Resultados','Lazo_completo_33_curvas');if ~isfolder(salida),mkdir(salida);endif ~isfolder(salida),mkdir(salida);end
dt=p.dt;N=ceil(500/dt);rng(1,'twister');
z=inicializarEstadoSoloTraccion([pista.centro(1,:)';0;0;0;0],p);
memEstimador=struct('xh',z(1:5),'P',eye(5)*1e-3);
med=sensores(z,struct('ax',z(7),'ay',z(8),'delta_equivalente',0),p,1);
memCapacidad=[];memLong=[];memLat=[];memAyudas=[];uAnterior=zeros(5,1);
ayudas=struct('ABS',true,'TC',true,'ESC',true);
h=struct('t',zeros(N,1),'z',zeros(N,24),'estimado',zeros(N,7), ...
 'medidas',nan(N,7),'covarianza',zeros(N,7),'capacidad_estimada',zeros(N,4), ...
 'fuerzas',zeros(N,4),'Fx',zeros(N,4),'Fy',zeros(N,4),'capacidad',zeros(N,4));
for nombre={'error','error_estimado','avance','distancia','vRef','delta','nominal','ABS','TC','ESC','proyeccion_cargas'},h.(nombre{1})=zeros(N,1);end
h.motivo='Límite de tiempo';avance=0;distancia=0;sPrev=0;idxReal=1;idxEst=1;
for k=1:N
 paso=dt;if k==1,paso=0;end
 [est,memEstimador]=estimadorEstado(med,uAnterior,p,memEstimador,paso);
 [cap,memCapacidad]=estimarCapacidadControl(est,uAnterior,p,memCapacidad,paso);
 est.zControl=adaptarEntradaControl(est,cap);
 [errorEst,sControl,idxEst]=proyectarLocal([est.X est.Y],pista,idxEst);
 vRef=sqrt(interp1([perfil.s;perfil.longitud],[perfil.v.^2;perfil.v(1)^2],sControl,'linear'));
 [delta,memLat]=controlLateral(est,pista.centro,p,memLat);
 integralAntes=0;if ~isempty(memLong),integralAntes=memLong.e_int;end
 [nominal,memLong]=controlLongitudinal(est,struct('v_set',vRef),p,memLong,dt);
 [f,memAyudas,info]=coordinarAyudasExtremo(est,nominal,delta,p,ayudas,memAyudas,dt);
 if info.ESC&&nominal>0,memLong.e_int=integralAntes;memLong.Fx=0;end
 % La verdad de planta se usa únicamente para integrar y evaluar.
 [errorReal,sReal,idxReal]=proyectarLocal(z(1:2)',pista,idxReal);
 ds=sReal-sPrev;if ds<-pista.longitud/2,ds=ds+pista.longitud;end
 if ds>pista.longitud/2,ds=ds-pista.longitud;end
 avance=avance+ds;sPrev=sReal;
 try
  [zn,tele]=pasoDinamicoSoloTraccion(z,[f(:);delta],p,dt);
 catch fallo
  h.motivo=['Fallo físico/numérico: ' fallo.identifier ' ' fallo.message];k=k-1;break;
 end
 assert(all(isfinite(zn))&&all(tele.Fzw>=0));
 h.t(k)=(k-1)*dt;h.z(k,:)=z';h.estimado(k,:)=[est.X est.Y est.psi est.vx est.vy est.ay est.delta];
 h.medidas(k,:)=[med.acc_x med.acc_y med.gyro med.gps_X med.gps_Y med.delta med.hay_gps];
 h.covarianza(k,:)=diag(memEstimador.P)';h.capacidad_estimada(k,:)=cap.Fmax_w;
 h.error(k)=errorReal;h.error_estimado(k)=errorEst;h.avance(k)=avance;h.distancia(k)=distancia;
 h.vRef(k)=vRef;h.delta(k)=delta;h.nominal(k)=nominal;h.fuerzas(k,:)=f;
 h.Fx(k,:)=tele.Fxw;h.Fy(k,:)=tele.Fyw;h.capacidad(k,:)=tele.Fmax_w;
 h.ABS(k)=info.ABS;h.TC(k)=info.TC;h.ESC(k)=info.ESC;h.proyeccion_cargas(k)=cap.cargas_proyectadas;
 if mod(k,10000)==0,fprintf('Lazo completo: %.1f s | %.1f%% | error %.2f m | estimacion %.2f m\n',h.t(k),100*avance/pista.longitud,errorReal,hypot(est.X-z(1),est.Y-z(2)));end
 if avance>=pista.longitud&&distancia>.95*pista.longitud,h.motivo='Vuelta completada';break;end
 if abs(errorReal)>15,h.motivo='Salida: error lateral superior a 15 m';break;end
 if abs(atan2(z(5),z(4)))>deg2rad(85)&&distancia>30,h.motivo='Trompo: deriva superior a 85 grados';break;end
 med=sensores(zn,tele,p,k+1);uAnterior=[f(:);delta];distancia=distancia+norm(zn(1:2)-z(1:2));z=zn;
end
assert(k>0);for campo=fieldnames(h)',v=h.(campo{1});if isnumeric(v)&&size(v,1)==N,h.(campo{1})=v(1:k,:);end;end
resumen=struct('finalizacion',h.motivo,'tiempo_s',h.t(end),'avance_pct',100*h.avance(end)/pista.longitud, ...
 'error_lateral_max_m',max(abs(h.error)),'rmse_posicion_estimador_m',sqrt(mean(sum((h.estimado(:,1:2)-h.z(:,1:2)).^2,2))), ...
 'ABS_s',sum(h.ABS)*dt,'TC_s',sum(h.TC)*dt,'ESC_s',sum(h.ESC)*dt,'proyecciones_carga',sum(h.proyeccion_cargas));
save(fullfile(salida,'Lazo_completo_33_curvas.mat'),'h','p','plan','pista','perfil','ayudas','resumen','-v7.3');
fid=fopen(fullfile(salida,'Resumen.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(resumen,PrettyPrint=true));fclose(fid);disp(resumen);

function [e,s,idx]=proyectarLocal(pos,pista,anterior)
n=numel(pista.s);ids=unique(mod((anterior-50:anterior+250)-1,n)+1);
rel=pos-pista.segmentoA(ids,:);q=sum(rel.*pista.segmentoD(ids,:),2)./pista.segmentoL(ids).^2;
q=max(0,min(1,q));pie=pista.segmentoA(ids,:)+q.*pista.segmentoD(ids,:);
[d,j]=min(sum((pos-pie).^2,2));
if d>400
    ids=(1:n)';rel=pos-pista.segmentoA;q=sum(rel.*pista.segmentoD,2)./pista.segmentoL.^2;
    q=max(0,min(1,q));pie=pista.segmentoA+q.*pista.segmentoD;[~,j]=min(sum((pos-pie).^2,2));
end
idx=ids(j);t=pista.segmentoD(idx,:)/pista.segmentoL(idx);v=pos-pie(j,:);
e=-t(2)*v(1)+t(1)*v(2);s=mod(pista.s(idx)+q(j)*pista.segmentoL(idx),pista.longitud);
end

function perfil=planificarExtremo(pista,vMax,p)
k=abs(pista.curvatura);v2=min(vMax^2,p.ayMax./max(k,1e-12));n=numel(v2);ds=pista.segmentoL;
for iter=1:200
 anterior=v2;
 for i=n:-1:1,j=mod(i,n)+1;v2(i)=min(v2(i),v2(j)+2*p.axFreno*ds(i));end
 for i=1:n,j=mod(i,n)+1;v2(j)=min(v2(j),v2(i)+2*p.axAcel*ds(i));end
 if max(abs(v2-anterior))<1e-9,break;end
end
assert(iter<200);perfil=struct('s',pista.s,'longitud',pista.longitud,'v',sqrt(v2));
end

