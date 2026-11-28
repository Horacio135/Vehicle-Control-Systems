base=fileparts(mfilename('fullpath'));raiz=fileparts(fileparts(base));addpath(raiz,fileparts(base),'-begin');salida=fullfile(raiz,'Resultados','Lazo_completo_33_curvas');if ~isfolder(salida),mkdir(salida);end
datos=fullfile(salida,'Lazo_completo_33_curvas.mat');if ~isfile(datos),datos=fullfile(raiz,'..','Results','Extreme_33_Corners','Complete_Loop','Lazo_completo_33_curvas.mat');end;d=load(datos);h=d.h;p=d.p;
assert(isequal(p.Q,diag([.001 .001 .00001 .003 .003].^2)));
assert(all([d.ayudas.ABS d.ayudas.TC d.ayudas.ESC]));
assert(any(h.medidas(:,7))&&any(~h.medidas(:,7)));
ruidoAx=std(h.medidas(:,1)-h.z(:,7));ruidoAy=std(h.medidas(:,2)-h.z(:,8));
ruidoGiro=std(h.medidas(:,3)-h.z(:,6));
idsGPS=logical(h.medidas(:,7));ruidoGPS=std(h.medidas(idsGPS,4:5)-h.z(idsGPS,1:2));assert(all(abs(ruidoGPS-p.gps_sigma)<.05)&&abs(ruidoGiro-p.gyro_sigma)<.001);
assert(any(abs(h.estimado(:,1:2)-h.z(:,1:2))>1e-6,'all'));
assert(all(isfinite(h.z),'all')&&all(isfinite(h.estimado),'all'));
assert(~any(any(h.fuerzas>0,2)&any(h.fuerzas<0,2)));
informe=struct('lazo_sensores_estimador_activo',true,'estado_perfecto_en_control',false, ...
 'Q',p.Q,'P0',eye(5)*1e-3,'semilla',1,'dt_s',p.dt,'GPS_cada_pasos',p.gps_cada, ...
 'sigma_ax_configurada',p.acc_sigma,'sigma_ay_configurada',p.acc_y_sigma,'sigma_GPS_observada',ruidoGPS, ...
 'residuo_medida_ax_frente_estado_std',ruidoAx,'residuo_medida_ay_frente_estado_std',ruidoAy, ...
 'ruido_giro_std_observado',ruidoGiro,'resumen',d.resumen);
fid=fopen(fullfile(salida,'Verificacion_lazo.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(informe,PrettyPrint=true));fclose(fid);disp(informe);
