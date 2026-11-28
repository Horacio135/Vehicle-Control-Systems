%% Circuito extremo:33 curvas, cuatro configuraciones de ayudas
base=fileparts(mfilename('fullpath'));raiz=fileparts(base);
for sub={'Dinamica','Control','Configuracion'},addpath(fullfile(raiz,sub{1}),'-begin');end
addpath(base,'-begin');
salida=fullfile(raiz,'Resultados','Circuito_extremo_33_curvas');if ~isfolder(salida),mkdir(salida);endif ~isfolder(salida),mkdir(salida);end
[p,~,plan]=configuracionSoloTraccion;p=configuracionESC(p);
cfg=struct('vMax',40,'tiempoMaximo',500,'errorAbandono',15,'ancho',6,'estadoPerfecto',true);
plan.ayMax=11;plan.axAcel=6;plan.axFreno=12;
pista=generarCircuitoExtremo;perfil=planificarExtremo(pista,cfg.vMax,plan);
graficarPista(pista,perfil,salida);
nombres={'Sin ayudas','Solo ABS','Solo TC','ABS + TC + ESC'};
ayudas=struct('ABS',{false,true,false,true},'TC',{false,false,true,true},'ESC',{false,false,false,true});
historias=cell(1,4);resumen=struct([]);
for caso=1:4
    fprintf('\nCASO%d: %s | longitud%.1fm |33 curvas\n',caso,nombres{caso},pista.longitud);
    h=simularCaso(p,pista,perfil,cfg,ayudas(caso));historias{caso}=h;
    traccion=h.nominal>0;frenada=h.nominal<0;
    r=struct('Caso',nombres{caso},'Finalizacion',h.motivo,'Tiempo_s',h.t(end), ...
        'Avance_pct',100*h.avance(end)/pista.longitud,'Distancia_m',h.distancia(end), ...
        'VelocidadMax_ms',max(h.z(:,4)),'BetaMax_deg',max(abs(rad2deg(h.beta))), ...
        'ErrorMax_m',max(abs(h.error)),'ErrorRMS_m',sqrt(mean(h.error.^2)), ...
        'TiempoTC_s',sum(h.TC)*p.dt,'TiempoABS_s',sum(h.ABS)*p.dt,'TiempoESC_s',sum(h.ESC)*p.dt, ...
        'TraccionEnCurva_s',sum(traccion&abs(h.curvatura)>.003)*p.dt, ...
        'FrenadaEnCurva_s',sum(frenada&abs(h.curvatura)>.003)*p.dt, ...
        'TiempoBloqueo_s',sum(any(h.bloqueo,2))*p.dt);
    if caso==1,resumen=r;else,resumen(caso)=r;end
    disp(r);save(fullfile(salida,sprintf('Caso_%d.mat',caso)),'h','p','cfg','ayudas','caso','-v7.3');
    writetable(struct2table(resumen),fullfile(salida,'Resumen.csv'));
end
save(fullfile(salida,'Comparacion_completa.mat'),'historias','resumen','nombres','p','cfg','plan','pista','perfil','ayudas','-v7.3');
graficarResultados(historias,nombres,pista,salida);

function h=simularCaso(p,pista,perfil,cfg,ayudas)
dt=p.dt;N=ceil(cfg.tiempoMaximo/dt);z=inicializarEstadoSoloTraccion([pista.centro(1,:)';0;0;0;0],p);
memLat=[];memLong=[];memAyudas=[];distancia=0;avance=0;sPrev=0;idxProy=1;
h=struct();
campos={'z','fuerzas','Fx','Fy','Fz','capacidad','kappa','bloqueo','frenoESC','factorABS'};
anchos=[24 4 4 4 4 4 4 4 4 4];
for j=1:numel(campos),h.(campos{j})=zeros(N,anchos(j));end
for nombre={'t','nominal','delta','vRef','error','avance','distancia','beta','TC','ABS','ESC','rRef','momento','curvatura','seleccion'}
    h.(nombre{1})=zeros(N,1);
end
h.motivo='Límite de tiempo';h.ayudas=ayudas;
for k=1:N
    [error,sActual,idxProy]=proyectarLocal(z(1:2).',pista,idxProy);
    ds=sActual-sPrev;if ds<-pista.longitud/2,ds=ds+pista.longitud;end
    if ds>pista.longitud/2,ds=ds-pista.longitud;end
    avance=avance+ds;sPrev=sActual;
    est=struct('X',z(1),'Y',z(2),'psi',z(3),'vx',z(4),'vy',z(5),'r',z(6), ...
        'ax',z(7),'ay',z(8),'zControl',z(1:20));
    [delta,memLat]=controlLateral(est,pista.centro,p,memLat);
    vRef=sqrt(interp1([perfil.s;perfil.longitud],[perfil.v.^2;perfil.v(1)^2],sActual,'linear'));
    if isempty(memLong),integralAntes=0;else,integralAntes=memLong.e_int;end
    [nominal,memLong]=controlLongitudinal(est,struct('v_set',vRef),p,memLong,dt);
    [f,memAyudas,info]=coordinarAyudasExtremo(est,nominal,delta,p,ayudas,memAyudas,dt);
    if info.ESC && nominal>0,memLong.e_int=integralAntes;memLong.Fx=0;end
    try
        [zn,tele]=pasoDinamicoSoloTraccion(z,[f(:);delta],p,dt);
    catch fallo
        h.motivo=['Límite físico/numerico: ' fallo.identifier ' ' fallo.message];
        fprintf('%s\n',h.motivo);k=k-1;break
    end
    assert(all(isfinite(zn))&&all(tele.Fzw>=0));
    h.t(k)=(k-1)*dt;h.z(k,:)=z';h.fuerzas(k,:)=f;h.nominal(k)=nominal;
    h.Fx(k,:)=tele.Fxw;h.Fy(k,:)=tele.Fyw;h.Fz(k,:)=tele.Fzw;h.capacidad(k,:)=tele.Fmax_w;
    h.kappa(k,:)=tele.kappa;h.bloqueo(k,:)=zn(17:20).';h.delta(k)=delta;
    h.vRef(k)=vRef;h.error(k)=error;h.avance(k)=avance;h.distancia(k)=distancia;
    h.beta(k)=atan2(z(5),z(4));h.TC(k)=info.TC;h.ABS(k)=info.ABS;h.ESC(k)=info.ESC;
    h.rRef(k)=info.rRef;h.momento(k)=info.momento;h.frenoESC(k,:)=info.frenoESC;
    h.factorABS(k,:)=info.factorABS;h.curvatura(k)=pista.curvatura(idxProy);h.seleccion(k)=info.seleccion;
    if mod(k,10000)==0,fprintf('t%.1fs avance%.1f%% error%.2fm v%.1fm/s\n',h.t(k),100*avance/pista.longitud,error,z(4));end
    if avance>=pista.longitud && distancia>.95*pista.longitud,h.motivo='Vuelta completada';break;end
    if abs(error)>cfg.errorAbandono,h.motivo='Salida: error lateral superior a15m';break;end
    if abs(h.beta(k))>deg2rad(85)&&distancia>30,h.motivo='Trompo: deriva superior a85grados';break;end
    distancia=distancia+norm(zn(1:2)-z(1:2));z=zn;
end
assert(k>0,'No hay muestras válidas.');
for nombre=fieldnames(h)'
    valor=h.(nombre{1});if isnumeric(valor)&&size(valor,1)==N,h.(nombre{1})=valor(1:k,:);end
end
end

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

function graficarPista(pista,perfil,salida)
fig=figure('Visible','off','Color','w','Position',[20 20 1400 850]);tiledlayout(2,1);
nexttile;plot(pista.bordeIzq(:,1),pista.bordeIzq(:,2),'Color',[.7 .7 .7]);hold on;
plot(pista.bordeDer(:,1),pista.bordeDer(:,2),'Color',[.7 .7 .7]);
scatter(pista.centro(:,1),pista.centro(:,2),5,perfil.v,'filled');axis equal;grid on;colorbar;
xlabel('X[m]');ylabel('Y[m]');title(sprintf('Circuito extremo:33 curvas, %.1fm, color=velocidad de referencia[m/s]',pista.longitud));
nexttile;plot(perfil.s,perfil.v);grid on;xlabel('Distancia de pista[m]');ylabel('Referencia[m/s]');
exportgraphics(fig,fullfile(salida,'Circuito_y_perfil.png'),'Resolution',150);close(fig);
writetable(pista.curvas,fullfile(salida,'Inventario_33_curvas.csv'));
writetable(pista.sectores,fullfile(salida,'Sectores.csv'));
end

function graficarResultados(historias,nombres,pista,salida)
colores=lines(4);fig=figure('Visible','off','Color','w','Position',[20 20 1400 900]);tiledlayout(2,2);
nexttile;plot(pista.centro(:,1),pista.centro(:,2),'k--');hold on;
for j=1:4,h=historias{j};plot(h.z(:,1),h.z(:,2),'Color',colores(j,:));end
axis equal;grid on;legend([{'Referencia'} nombres]);xlabel('X[m]');ylabel('Y[m]');
nexttile;hold on;for j=1:4,h=historias{j};plot(h.t,h.z(:,4),'Color',colores(j,:));end;grid on;legend(nombres);xlabel('Tiempo[s]');ylabel('Velocidad[m/s]');
nexttile;hold on;for j=1:4,h=historias{j};plot(h.t,rad2deg(h.beta),'Color',colores(j,:));end;grid on;legend(nombres);xlabel('Tiempo[s]');ylabel('Deriva[grados]');
nexttile;hold on;for j=1:4,h=historias{j};plot(h.t,h.error,'Color',colores(j,:));end;grid on;legend(nombres);xlabel('Tiempo[s]');ylabel('Error lateral[m]');
exportgraphics(fig,fullfile(salida,'Comparacion_cuatro_casos.png'),'Resolution',150);close(fig);
end
