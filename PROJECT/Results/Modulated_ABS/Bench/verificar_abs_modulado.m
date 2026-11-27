function verificar_abs_modulado
base=fileparts(mfilename('fullpath'));paquete=fullfile(fileparts(fileparts(fileparts(base))),'Current State');
for carpeta={'Dinamica','Control','Sensores','Estimacion','Configuracion'},addpath(fullfile(paquete,carpeta{1}),'-begin');end
addpath(base,'-begin');[p,~,~]=configuracionSoloTraccion;dt=p.dt;
z=inicializarEstadoSoloTraccion([0;0;0;25;0;0],p);cap=p.muw.*p.Fzw0;
N=251;t=(0:N-1)'*dt;factores=zeros(N,4);fuerzas=zeros(N,4);fases=zeros(N,4);mem=[];
for k=1:N
 [f,info,mem]=controlABS(-1.3*cap,z(1:20),p,mem,dt);
 factores(k,:)=info.factorAplicado;fuerzas(k,:)=f;fases(k,:)=mem.fase;
 assert(all(f<=0)&&all(abs(f)<=.98*cap+1e-9));
end
assert(max(abs(factores(1,:)-.98))<1e-12);
assert(max(abs(factores(16,:)-.80))<1e-12); % 30 ms después del disparo.
assert(max(abs(factores(21,:)-.80))<1e-12); % Fin del mantenimiento a 40 ms.
assert(max(abs(factores(51,:)-.98))<1e-12); % Fin de recarga a 100 ms.
assert(all(diff(factores(:,1))>=-6*dt-1e-10));
assert(all(diff(factores(:,1))<=3*dt+1e-10));
assert(max(abs(factores(1:51,1)-factores(51:101,1)))<1e-10);
% Misma evolución con dt=1 ms: memoria causal y duración invariantes.
m=[];fino=zeros(501,1);
for k=1:501,[~,i,m]=controlABS(-1.3*cap,z(1:20),p,m,.001);fino(k)=i.factorAplicado(1);end
assert(max(abs(fino(1:2:end)-factores(:,1)))<1e-10);
% Canales independientes, y solicitudes positivas/nulas inalteradas.
m=[];
for k=1:60
 [f,i,m]=controlABS([-1.2*cap(1) -.5*cap(2) 70 0],z(1:20),p,m,dt);
 assert(f(2)==-.5*cap(2)&&f(3)==70&&f(4)==0);
 assert(all(m.fase(2:4)==0));
end
[f,~,m]=controlABS([0 0 0 0],z(1:20),p,m,dt);assert(all(f==0)&&all(m.fase==0));
[f,~,m]=controlABS([-2*cap(1);0;0;0],z(1:20),p,m,dt);assert(isequal(size(f),[4 1]));
[sinFreno,~]=controlABS([0 0 50 50],z(1:20),p);assert(isequal(sinFreno,[0 0 50 50]));
% Una llamada antigua sin memoria no puede fingir regulación temporal.
errorMemoria=false;
try,[~,~]=controlABS(-cap,z(1:20),p);catch e,errorMemoria=strcmp(e.identifier,'controlABS:MemoriaNecesaria');end
assert(errorMemoria);
% controlTotal conserva y cancela la memoria al cambiar de modo/emergencia.
est=struct('X',0,'Y',0,'psi',0,'vx',20,'zControl',z(1:20));trayectoria=[(0:300)' zeros(301,1)];m=[];
for k=1:200,[~,~,m,i]=controlTotal(est,struct('v_set',0),trayectoria,[],p,m,dt);end
assert(i.ABSActivo&&isstruct(m.abs));
for k=1:100,[~,~,m,i]=controlTotal(est,struct('v_set',30),trayectoria,[],p,m,dt);end
assert(i.TCActivo&&isempty(m.abs));
m.emergencia.latched=true;m.abs=mem;
[~,~,m,i]=controlTotal(est,struct('v_set',0),trayectoria,[],p,m,dt);
assert(i.emergenciaActiva&&~i.ABSActivo&&~i.TCActivo&&isempty(m.abs));
% La dinámica permanece congelada. Comparación recta para evaluar el efecto
% de la modulación; no se afirma que mejore la distancia del limitador ideal.
[hNuevo,rNuevo]=frenadaRecta(p,true);[hAnterior,rAnterior]=frenadaRecta(p,false);
salida=base;if ~isfolder(salida),mkdir(salida);end
save(fullfile(salida,'verificacion_ABS.mat'),'t','factores','fuerzas','fases','hNuevo','hAnterior','p');
resumen=struct('perfil_098_a_080_verificado',true,'descarga_ms',30,'mantenimiento_ms',10,'recarga_ms',60,'periodo_nominal_ms',100, ...
 'pendiente_descarga_normalizada_s_1',-6,'pendiente_recarga_normalizada_s_1',3,'independencia_ruedas',true, ...
 'invariancia_dt_1_y_2_ms',true,'sin_intervencion_en_traccion',true,'memoria_integrada_en_controlTotal',true, ...
 'reinicio_al_soltar_traccion_emergencia',true,'memoria_explicita_exigida',true,'frenada_nueva',rNuevo,'frenada_anterior',rAnterior);
fid=fopen(fullfile(salida,'verificacion_ABS.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(resumen,PrettyPrint=true));fclose(fid);
f=figure('Visible','off','Color','w','Position',[20 20 1152 720]);tiledlayout(2,2,'TileSpacing','compact');
a=nexttile;plot(a,t,factores(:,1),'LineWidth',1.5);yline(a,.98,'k--');yline(a,.8,'k:');grid(a,'on');xlabel(a,'Tiempo [s]');ylabel(a,'Factor de capacidad');title(a,'Perfil ABS: 30 ms descarga + 10 ms mantener + 60 ms recarga');ylim(a,[.75 1.02]);
a=nexttile;plot(a,t,abs(fuerzas));grid(a,'on');xlabel(a,'Tiempo [s]');ylabel(a,'Fuerza solicitada corregida [N]');legend(a,'DI','DD','TI','TD');title(a,'Capacidad constante; demanda superior al límite');
a=nexttile;plot(a,hAnterior.t,hAnterior.v,'k--',hNuevo.t,hNuevo.v,'b');grid(a,'on');xlabel(a,'Tiempo [s]');ylabel(a,'vx [m/s]');legend(a,'Limitador anterior al 100%','ABS modulado');title(a,'Frenada recta desde 25 m/s, sin sensores/estimador');
a=nexttile;plot(a,hNuevo.t,hNuevo.f(:,1:2),'LineWidth',1);grid(a,'on');xlabel(a,'Tiempo [s]');ylabel(a,'Fuerza de frenada delantera [N]');legend(a,'DI','DD');title(a,'Modulación con transferencia de carga real del modelo');
sgtitle('Verificación del ABS modulado por fuerza | Parámetros iniciales, no calibración de competición');
exportgraphics(f,fullfile(salida,'Perfil_ABS_modulado.png'),'Resolution',150);savefig(f,fullfile(salida,'Perfil_ABS_modulado.fig'));close(f);
disp(resumen);
end

function [h,r]=frenadaRecta(p,nuevo)
dt=p.dt;z=inicializarEstadoSoloTraccion([0;0;0;25;0;0],p);mem=[];N=ceil(10/dt);
h=struct('t',nan(N,1),'v',nan(N,1),'x',nan(N,1),'f',nan(N,4));
for k=1:N
 demanda=-p.Fx_brake_max*(1-exp(-(k-1)*dt/.02))*p.brake_w;
 if nuevo,[f,~,mem]=controlABS(demanda,z(1:20),p,mem,dt);else,[f,~]=controlABSAnterior(demanda,z(1:20),p);end
 [zn,tele]=pasoDinamicoSoloTraccion(z,[f(:);0],p,dt);
 assert(all(isfinite(zn))&&all(tele.Fzw>=0));
 h.t(k)=(k-1)*dt;h.v(k)=z(4);h.x(k)=z(1);h.f(k,:)=f;
 z=zn;if z(4)<.1,break;end
end
for nombre=fieldnames(h)',h.(nombre{1})=h.(nombre{1})(1:k,:);end
assert(z(4)<.1,'El vehículo no se detuvo en 10 s.');
r=struct('tiempo_hasta_01_ms_s',h.t(end)+dt,'distancia_hasta_01_ms_m',z(1),'fuerza_frenada_max_N',max(abs(h.f),[],'all'));
end
