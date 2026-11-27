%% Lazo completo actual: ABS off/on, con TC activo en ambos
% Sensores y estimador activos; Q reducida mantenida. Física y TC intactos.
% La comparación modifica únicamente p.abs.habilitado.
carpeta=fileparts(mfilename('fullpath'));
for sub={'Dinamica','Control','Sensores','Estimacion','Configuracion'}
    addpath(fullfile(carpeta,sub{1}),'-begin');
end
addpath(carpeta,'-begin'); % Adaptaciones externas; bloques originales intactos.
comprobarBloques(carpeta);
assert(strcmpi(which('estimarCapacidadControl'),fullfile(carpeta,'estimarCapacidadControl.m')));
assert(strcmpi(which('adaptarEntradaControl'),fullfile(carpeta,'adaptarEntradaControl.m')));
[p,cfg,plan]=configuracionIntegracion;
pista=generarCircuitoHibrido(cfg);
perfil=planificarVelocidad(pista,cfg.velocidadMaxima,plan);
% Comparación aislada del ABS: TC permanece habilitado en ambos casos.
pSinABS=p;pSinABS.abs.habilitado=false;
pConABS=p;pConABS.abs.habilitado=true;
sinControl=simularLazo(true,pSinABS,pista,perfil,cfg);
conControl=simularLazo(true,pConABS,pista,perfil,cfg);
casos={sinControl,conControl};
nombres={'ABS desactivado, TC activado','ABS y TC activados'};
referenciaPerfecta=[];
if cfg.compararEstadoPerfecto
    cfgPerfecta=cfg;cfgPerfecta.usarSensoresEstimador=false;
    referenciaPerfecta=simularLazo(true,p,pista,perfil,cfgPerfecta);
    casos{end+1}=referenciaPerfecta;nombres{end+1}='Referencia con estado perfecto';
end
resumen=table(string(nombres(:)),cellfun(@(h)h.t(end),casos).', ...
    cellfun(@(h)max(abs(h.error)),casos).', ...
    cellfun(@(h)sqrt(mean(h.error.^2)),casos).', ...
    cellfun(@(h)max(h.z(:,4)),casos).', ...
    cellfun(@(h)h.avance(end)/pista.longitud,casos).', ...
    cellfun(@(h)sqrt(mean(sum(h.error_estimacion(:,1:2).^2,2))),casos).', ...
    cellfun(@(h)sqrt(mean(h.error_estimacion(:,4).^2)),casos).', ...
    cellfun(@(h)sqrt(mean(h.error_estimacion(:,5).^2)),casos).', ...
    cellfun(@(h)sum(h.cargas_proyectadas),casos).', ...
    string(cellfun(@(h)h.motivo,casos,'UniformOutput',false)).', ...
    'VariableNames',{'Caso','Tiempo_s','ErrorMax_m','ErrorRMS_m','VelocidadMax_ms', ...
    'Vueltas','RMSE_posicion_estimador_m','RMSE_vx_ms','RMSE_vy_ms','Proyecciones_carga','Finalizacion'});
disp(resumen);
if cfg.guardarResultados
    if ~isfolder(cfg.carpetaResultados),mkdir(cfg.carpetaResultados);end
    save(fullfile(cfg.carpetaResultados,'lazo_completo.mat'), ...
        'sinControl','conControl','referenciaPerfecta','p','pSinABS','pConABS','cfg','plan','pista','perfil','resumen');
    writetable(resumen,fullfile(cfg.carpetaResultados,'resumen.csv'));
end
fig=figure('Visible','off','Color','w','Position',[50 50 1300 850]);tiledlayout(2,2);
colores=[.85 .3 .1;0 .45 .74;.4 .4 .4];
nexttile;plot(pista.centro(:,1),pista.centro(:,2),'k--');hold on;
for j=1:numel(casos),plot(casos{j}.z(:,1),casos{j}.z(:,2),'Color',colores(j,:));end
axis equal;grid on;legend([{'Centro'} nombres],'Location','best');xlabel('X [m]');ylabel('Y [m]');
nexttile;hold on;for j=1:numel(casos),plot(casos{j}.t,casos{j}.z(:,4),'Color',colores(j,:));end
grid on;legend(nombres,'Location','best');xlabel('Tiempo [s]');ylabel('Velocidad real [m/s]');
nexttile;hold on;for j=1:numel(casos),plot(casos{j}.t,casos{j}.error,'Color',colores(j,:));end
grid on;legend(nombres,'Location','best');xlabel('Tiempo [s]');ylabel('Error de trayectoria real [m]');
nexttile;hold on;for j=1:2,plot(casos{j}.t,vecnorm(casos{j}.error_estimacion(:,1:2),2,2),'Color',colores(j,:));end
grid on;legend(nombres(1:2),'Location','best');xlabel('Tiempo [s]');ylabel('Error de posición estimada [m]');
if cfg.guardarResultados,exportgraphics(fig,fullfile(cfg.carpetaResultados,'comparacion_lazo.png'),'Resolution',150);end
close(fig);
fig=figure('Visible','off','Color','w','Position',[50 50 1300 850]);tiledlayout(2,2);
nexttile;plot(conControl.t,[conControl.z(:,4) conControl.estimado(:,4)]);grid on;
legend('Real','Estimada');xlabel('Tiempo [s]');ylabel('vx [m/s]');
nexttile;plot(conControl.t,[conControl.z(:,5) conControl.estimado(:,5)]);grid on;
legend('Real','Estimada');xlabel('Tiempo [s]');ylabel('vy [m/s]');
nexttile;plot(conControl.t,[conControl.z(:,8) conControl.medidas(:,2) conControl.estimado(:,6)]);grid on;
legend('Real','Medida','Estimada');xlabel('Tiempo [s]');ylabel('ay [m/s^2]');
nexttile;plot(conControl.t,conControl.capacidad_estimada);grid on;
legend('DI','DD','TI','TD');xlabel('Tiempo [s]');ylabel('Capacidad predicha para control [N]');
if cfg.guardarResultados,exportgraphics(fig,fullfile(cfg.carpetaResultados,'estimacion_y_capacidad.png'),'Resolution',150);end
close(fig);

function h=simularLazo(activar,p,pista,perfil,cfg)
N=ceil(cfg.tiempoMaximo/p.dt);memControl=[];memEstimador=[];memCapacidad=[];
uAnterior=zeros(5,1);med=[];rng(1,'twister');
% Único acceso del estimador al estado inicial conocido del escenario.
z=inicializarEstadoSoloTraccion([pista.centro(1,:)';atan2(pista.tangente(1,2),pista.tangente(1,1));0;0;0],p);
if cfg.usarSensoresEstimador
    memEstimador=struct('xh',z(1:5),'P',eye(5)*1e-3);
    med=sensores(z,struct('ax',z(7),'ay',z(8)),p,1);
end
h=struct();campos={'z','FxSolicitadas','FxFinales','Fxw','Fyw','capacidad','estimado', ...
    'error_estimacion','medidas','capacidad_estimada','eta_brake_predicha','covarianza','delta_ruedas','kappa','omega'};
anchos=[24 4 4 4 4 4 7 6 7 4 4 7 4 4 4];
for j=1:numel(campos),h.(campos{j})=nan(N,anchos(j));end
for nombre={'t','error','error_estimado','distancia','avance','vref','modos','delta', ...
        'potenciaMotor','disipacion','cargas_proyectadas','s_control'}
    h.(nombre{1})=zeros(N,1);
end
h.motivo='Límite de tiempo';avance=0;sPrev=0;distancia=0;
for k=1:N
    if cfg.usarSensoresEstimador
        paso=p.dt;if k==1,paso=0;end
        [est,memEstimador]=estimadorEstado(med,uAnterior,p,memEstimador,paso);
        [cap,memCapacidad]=estimarCapacidadControl(est,uAnterior,p,memCapacidad,paso);
        est.zControl=adaptarEntradaControl(est,cap);
        % zControl es solo el formato heredado; sus memorias son del control.
        h.capacidad_estimada(k,:)=cap.Fmax_w;
        h.eta_brake_predicha(k,:)=cap.eta_brake;
        h.cargas_proyectadas(k)=cap.cargas_proyectadas;
        h.covarianza(k,:)=diag(memEstimador.P).';
        h.medidas(k,:)=[med.acc_x med.acc_y med.gyro med.gps_X med.gps_Y med.delta med.hay_gps];
    else
        % Comparación explícita: referencia perfecta, fuera del lazo sensorizado.
        est=struct('X',z(1),'Y',z(2),'psi',z(3),'vx',z(4),'vy',z(5), ...
            'r',z(6),'ax',z(7),'ay',z(8),'delta',uAnterior(5),'zControl',z(1:20));
    end
    % Control y planificación reciben SOLO posición/velocidad estimadas.
    [errorEstimado,sControl]=proyectarCircuito([est.X est.Y],pista);
    ref.v_set=consultarPerfil(perfil,sControl);
    if activar
        [fuerzas,delta,memControl,info]=controlTotal(est,ref,pista.centro,cfg.obstaculo,p,memControl,p.dt);
        nominal=info.FxSolicitada;
        modo=double(info.TCActivo)+2*double(info.ABSActivo)+3*double(info.emergenciaActiva);
    else
        if isempty(memControl),memControl=struct('lateral',[],'longitudinal',[]);end
        [delta,memControl.lateral]=controlLateral(est,pista.centro,p,memControl.lateral);
        [nominal,memControl.longitudinal]=controlLongitudinal(est,ref,p,memControl.longitudinal,p.dt);
        if nominal>=0,fuerzas=nominal*p.drive_w;else,fuerzas=nominal*p.brake_w;end
        modo=0;
    end
    % Desde aquí la verdad se usa para evaluar y evolucionar la planta.
    [errorReal,sActual]=proyectarCircuito(z(1:2)',pista);
    ds=sActual-sPrev;
    if ds < -pista.longitud/2,ds=ds+pista.longitud;end
    if ds > pista.longitud/2,ds=ds-pista.longitud;end
    avance=avance+ds;sPrev=sActual;
    if nominal>=0,solicitadas=nominal*p.drive_w;else,solicitadas=nominal*p.brake_w;end
    if any(fuerzas>0),assert(fuerzas(3)==fuerzas(4)&&all(fuerzas(1:2)==0));end
    assert(~(any(fuerzas>0)&&any(fuerzas<0)));
    [siguiente,tele]=pasoDinamicoSoloTraccion(z,[fuerzas(:);delta],p,p.dt);
    assert(isreal(siguiente)&&all(isfinite(siguiente))&&all(tele.Fzw>=0),'Estado físico inválido.');
    assert(isequal(tele.Fxw_sol,fuerzas));
    h.t(k)=(k-1)*p.dt;h.z(k,:)=z';h.error(k)=errorReal;h.error_estimado(k)=errorEstimado;
    h.distancia(k)=distancia;h.avance(k)=avance;h.s_control(k)=sControl;h.vref(k)=ref.v_set;
    h.modos(k)=modo;h.delta(k)=delta;h.delta_ruedas(k,:)=tele.delta_w;
    h.FxSolicitadas(k,:)=solicitadas;h.FxFinales(k,:)=fuerzas;
    h.Fxw(k,:)=tele.Fxw;h.Fyw(k,:)=tele.Fyw;h.capacidad(k,:)=tele.Fmax_w;
    h.kappa(k,:)=tele.kappa;h.omega(k,:)=tele.omega;
    h.potenciaMotor(k)=tele.potenciaMotor;h.disipacion(k)=tele.potenciaDisipada;
    h.estimado(k,:)=[est.X est.Y est.psi est.vx est.vy est.ay est.delta];
    diferencia=[est.X;est.Y;est.psi;est.vx;est.vy;est.r]-z(1:6);
    diferencia(3)=atan2(sin(diferencia(3)),cos(diferencia(3)));
    h.error_estimacion(k,:)=diferencia.';
    if mod(k,5000)==0,fprintf('Estimado %d, TC activo, ABS %d: %.1f s, avance %.1f %%, error %.2f m\n',cfg.usarSensoresEstimador,p.abs.habilitado,h.t(k),100*avance/pista.longitud,errorReal);end
    if avance>=pista.longitud&&distancia>.9*pista.longitud,h.motivo='Vuelta completada';break;end
    if abs(errorReal)>cfg.errorAbandono,h.motivo='Error lateral mayor de 10 m';break;end
    if cfg.usarSensoresEstimador,med=sensores(siguiente,tele,p,k+1);end
    uAnterior=[fuerzas(:);delta];
    distancia=distancia+norm(siguiente(1:2)-z(1:2));z=siguiente;
end
for nombre=fieldnames(h)'
    if isnumeric(h.(nombre{1})),h.(nombre{1})=h.(nombre{1})(1:k,:);end
end
fprintf('Estimado %d, TC activo, ABS %d: %s, %.3f s, error máximo %.3f m.\n',cfg.usarSensoresEstimador,p.abs.habilitado,h.motivo,h.t(end),max(abs(h.error)));
end

function pista=generarCircuitoHibrido(cfg)
vertices=[0 0;180 0;180 60;155 60;155 40;130 40;130 75; ...
    100 75;100 57;78 57;78 100;0 100; ...
    40*cosd(250) 100+40*sind(250);-40 30];
radios=[12 4 12 6 5 5 9 8 5 5 10 3.5 10 12]';
n=size(vertices,1); entrada=zeros(n,2); salida=entrada; centros=entrada;
angulos=zeros(n,1); giros=zeros(n,1);
for i=1:n
    ant=mod(i-2,n)+1; sig=mod(i,n)+1;
    a=vertices(i,:)-vertices(ant,:); a=a/norm(a);
    b=vertices(sig,:)-vertices(i,:); b=b/norm(b);
    giros(i)=atan2(a(1)*b(2)-a(2)*b(1),dot(a,b));
    d=radios(i)*tan(abs(giros(i))/2);
    entrada(i,:)=vertices(i,:)-d*a; salida(i,:)=vertices(i,:)+d*b;
    centros(i,:)=entrada(i,:)+sign(giros(i))*radios(i)*[-a(2) a(1)];
    angulos(i)=atan2(entrada(i,2)-centros(i,2),entrada(i,1)-centros(i,1));
end
xy=salida(1,:); tangente=[1 0]; curvatura=0;
for iter=1:n
    i=mod(iter,n)+1; ant=mod(i-2,n)+1;
    desde=salida(ant,:); hasta=entrada(i,:);
    tramo=hasta-desde; longitud=norm(tramo); direccion=tramo/longitud;
    assert(dot(tramo,vertices(i,:)-vertices(ant,:))>0,'Corner radii encroach on a straight segment.');
    np=max(1,ceil(longitud/cfg.pasoTrayectoria));
    q=(1:np)'/np;
    xy=[xy;desde+q.*tramo];
    tangente=[tangente;repmat(direccion,np,1)];
    curvatura=[curvatura;zeros(np,1)];
    np=max(2,ceil(abs(giros(i))*radios(i)/cfg.pasoTrayectoria));
    theta=angulos(i)+giros(i)*(1:np)'/np;
    xy=[xy;centros(i,:)+radios(i)*[cos(theta) sin(theta)]];
    tangente=[tangente;sign(giros(i))*[-sin(theta) cos(theta)]];
    curvatura=[curvatura;repmat(sign(giros(i))/radios(i),np,1)];
end
assert(norm(xy(end,:)-xy(1,:))<1e-9,'The circuit does not close.');
xy(end,:)=[]; tangente(end,:)=[]; curvatura(end)=[];
normal=[-tangente(:,2) tangente(:,1)];
s=[0;cumsum(vecnorm(diff(xy),2,2))];
longitud=s(end)+norm(xy(end,:)-xy(1,:));
izq=xy+cfg.ancho/2*normal; der=xy-cfg.ancho/2*normal;
sConos=(0:cfg.pasoConos:longitud-1e-9)';
pista.centro=xy; pista.s=s; pista.longitud=longitud;
pista.tangente=tangente; pista.curvatura=curvatura;
pista.bordeIzq=izq; pista.bordeDer=der;
pista.conoIzq=interp1([s;longitud],[izq;izq(1,:)],sConos);
pista.conoDer=interp1([s;longitud],[der;der(1,:)],sConos);
pista.segmentoA=xy; pista.segmentoB=circshift(xy,-1,1);
pista.segmentoD=pista.segmentoB-pista.segmentoA;
pista.segmentoL=vecnorm(pista.segmentoD,2,2);
sentido=repmat("izquierda",n,1); sentido(giros<0)="derecha";
tipo=repmat("exterior",n,1); tipo(giros<0)="interior";
pista.curvas=table((1:n)',radios,sentido,tipo,centros(:,1),centros(:,2), ...
    'VariableNames',{'Curva','Radio_m','Sentido','Tipo','CentroX_m','CentroY_m'});
pista.curvas.Angulo_deg=abs(rad2deg(giros));
pista.rectaAntes90_m=norm(entrada(2,:)-salida(1,:));
pista.chicane=[4 5];
end

function [ect,sActual,idx]=proyectarCircuito(pos,pista)
rel=pos-pista.segmentoA;
q=sum(rel.*pista.segmentoD,2)./pista.segmentoL.^2;
q=min(max(q,0),1);
pie=pista.segmentoA+q.*pista.segmentoD;
[~,idx]=min(sum((pos-pie).^2,2));
d=pos-pie(idx,:); tang=pista.segmentoD(idx,:)/pista.segmentoL(idx);
ect=-tang(2)*d(1)+tang(1)*d(2);
sActual=mod(pista.s(idx)+q(idx)*pista.segmentoL(idx),pista.longitud);
end

function perfil=planificarVelocidad(pista,vMax,cfg)
validateattributes(vMax,{'numeric'},{'scalar','positive','finite'});
validateattributes(cfg.ayMax,{'numeric'},{'scalar','positive','finite'});
validateattributes(cfg.axAcel,{'numeric'},{'scalar','positive','finite'});
validateattributes(cfg.axFreno,{'numeric'},{'scalar','positive','finite'});
kappa=abs(pista.curvatura(:)); ds=pista.segmentoL(:); n=numel(kappa);
vLim=vMax*ones(n,1); curva=kappa>1e-9;
vLim(curva)=min(vMax,sqrt(cfg.ayMax./kappa(curva)));
v2=vLim.^2;
converge=false;
for iter=1:200
    anterior=v2;
    for i=n:-1:1
        j=mod(i,n)+1;
        v2(i)=min(v2(i),v2(j)+2*cfg.axFreno*ds(i));
    end
    for i=1:n
        j=mod(i,n)+1;
        v2(j)=min(v2(j),v2(i)+2*cfg.axAcel*ds(i));
    end
    if max(abs(v2-anterior))<1e-10, converge=true; break; end
end
assert(converge,'The periodic speed profile did not converge.');
perfil.v=sqrt(max(v2,0)); perfil.vLimCurvatura=vLim;
perfil.vMax=vMax; perfil.iteraciones=iter;
perfil.axTramo=(circshift(v2,-1)-v2)./(2*ds);
perfil.ayEstimada=v2.*kappa;
perfil.s=pista.s; perfil.longitud=pista.longitud;
end

function v=consultarPerfil(perfil,s)
s=mod(s,perfil.longitud);
v=sqrt(max(0,interp1([perfil.s;perfil.longitud], ...
    [perfil.v.^2;perfil.v(1)^2],s,'linear')));
end


function comprobarBloques(carpeta)
grupos={{'Dinamica',{'pasoDinamicoSoloTraccion','pasoFrenadoAnterior','fuerzaTractoraPacejka','inicializarEstadoSoloTraccion','angulosAckermann'}}, ...
    {'Control',{'controlTotal','controlLateral','controlLongitudinal','controlTraccion','controlABS','controlEmergencia'}}, ...
    {'Sensores',{'sensores'}},{'Estimacion',{'estimadorEstado'}}, ...
    {'Configuracion',{'configuracionSoloTraccion','parametrosSoloTraccion','configuracionIntegracion'}}};
for j=1:numel(grupos)
    grupo=grupos{j};nombres=grupo{2};
    for k=1:numel(nombres)
        esperado=fullfile(carpeta,grupo{1},[nombres{k} '.m']);
        assert(strcmpi(which(nombres{k}),esperado), ...
            'Otra versión del bloque %s está sustituyendo la copia de estado actual.',nombres{k});
    end
end
end