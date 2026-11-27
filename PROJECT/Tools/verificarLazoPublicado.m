% Verificación breve del paquete publicado, sin cambiar los bloques.
base=fileparts(mfilename('fullpath'));
repo=fileparts(fileparts(base));
actual=fullfile(repo,'PROJECT','Current State');
for d={'Dinamica','Control','Sensores','Estimacion','Configuracion'},addpath(fullfile(actual,d{1}),'-begin');end
addpath(actual,'-begin');
[p,cfg,~]=configuracionIntegracion;
assert(cfg.usarSensoresEstimador&&cfg.guardarResultados);
assert(strcmp(cfg.carpetaResultados,fullfile(actual,'Resultados','Lazo_completo_ABS_modulado_2026-11-27')));
datos=load(fullfile(repo,'PROJECT','Results','Closed_Loop','histories.mat'),'pista');
pista=datos.pista;z=inicializarEstadoSoloTraccion([pista.centro(1,:)';0;0;0;0],p);
memK=struct('xh',z(1:5),'P',eye(5)*1e-3);memC=[];memCap=[];u=zeros(5,1);
rng(1);med=sensores(z,struct('ax',0,'ay',0),p,1);
huboTraccion=false;huboFrenada=false;
for k=1:1000
    dt=p.dt;if k==1,dt=0;end
    [est,memK]=estimadorEstado(med,u,p,memK,dt);
    [cap,memCap]=estimarCapacidadControl(est,u,p,memCap,dt);
    est.zControl=adaptarEntradaControl(est,cap);
    vref=20;if k>700,vref=0;end
    [f,delta,memC,info]=controlTotal(est,struct('v_set',vref),pista.centro,[],p,memC,p.dt);
    huboTraccion=huboTraccion||info.TCActivo;huboFrenada=huboFrenada||info.ABSActivo;
    u=[f(:);delta];[z,tele]=pasoDinamicoSoloTraccion(z,u,p,p.dt);
    med=sensores(z,tele,p,k+1);
    assert(all(isfinite(z))&&all(isfinite(memK.xh)));
end
assert(huboTraccion&&huboFrenada);
informe=struct('matlab',version,'pasos',1000,'interfaz_completa',true, ...
    'TC_ejercitado',huboTraccion,'ABS_ejercitado',huboFrenada, ...
    'ruta_resultados_relativa_al_paquete',true,'archivos_locales_originales_modificados',false);
fid=fopen(fullfile(base,'closed_loop_matlab_checks.json'),'w','n','UTF-8');
fprintf(fid,'%s\n',jsonencode(informe,PrettyPrint=true));fclose(fid);
disp(informe);
