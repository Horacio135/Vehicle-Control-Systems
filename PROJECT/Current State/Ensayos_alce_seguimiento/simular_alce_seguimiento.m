function simular_alce_seguimiento
% Esquivas agresivas con Pure Pursuit y referencia constante de rapidez.
% Sin sensores, estimador ni ESC. Sin frenada solicitada. Bloques intactos.
base=fileparts(fileparts(mfilename('fullpath')));addpath(genpath(base));
salida=fullfile(fileparts(base),'Results','Moose_Test_02');
if ~isfolder(salida),mkdir(salida);end
[p,~,~]=configuracionSoloTraccion;dt=p.dt;
% Parametrización del conductor: PI de mantenimiento, sin modo pedal a fondo.
p.conductor.errorActivar=Inf;
archivos=[dir(fullfile(base,'Dinamica','*.m'));dir(fullfile(base,'Control','*.m'));dir(fullfile(base,'Configuracion','*.m'))];
procedencia=struct('archivo',{},'sha256',{});
for j=1:numel(archivos)
 nombre=fullfile(archivos(j).folder,archivos(j).name);
 procedencia(j).archivo=nombre;
 procedencia(j).sha256=hashArchivo(nombre);
end
casos=struct('nombre',{},'tipo',{},'velocidad',{},'longitud',{},'amplitud',{},'frecuencia',{});
vel=20;long=10;
for j=1:1
 casos(j)=struct('nombre',sprintf('Alce_PP_%02d',2),'tipo','alce','velocidad',vel(j),'longitud',long(j),'amplitud',4,'frecuencia',0);
end
historias=cell(1,1);resumen=struct([]);
for j=1:1
 c=casos(j);inicioX=40;
 T=(inicioX+2*c.longitud+10+50)/c.velocidad+1;
 t=(0:dt:T)';n=numel(t);z=inicializarEstadoSoloTraccion([0;0;0;c.velocidad;0;0],p);
 x=(0:.1:300)';y=zeros(size(x));
 for a=1:numel(x),[y(a),~,~]=perfilAlce(x(a)-inicioX,c.longitud,c.amplitud);end
 trayectoria=[x y];
 resistencia=.5*p.rho*p.Cd*p.Af*c.velocidad^2+p.fr*p.m*p.g;
 memLong=struct('Fx',resistencia,'modo',0,'e_int',resistencia/p.Ki_v);memLat=[];
 h=struct('t',t,'z',nan(n,24),'delta',nan(n,1),'Fx',nan(n,4),'Fy',nan(n,4), ...
 'Fz',nan(n,4),'capacidad',nan(n,4),'alpha',nan(n,4),'kappa',nan(n,4), ...
 'ref',trayectoria,'rapidez',nan(n,1),'fuerzaPrevia',nan(n,1),'intervencionTC',false(n,1),'intervencionABS',false(n,1), ...
 'fuerzasSolicitadas',zeros(n,4),'config',c,'motivo','Finalizado','p',p);
 for k=1:n
  est=struct('X',z(1),'Y',z(2),'psi',z(3),'vx',z(4));
  [delta,memLat]=controlLateral(est,trayectoria,p,memLat);
  % El PI recibe rapidez real en su canal escalar de velocidad.
  estLong=est;estLong.vx=hypot(z(4),z(5));
  [solicitada,memLong]=controlLongitudinal(estLong,struct('v_set',c.velocidad),p,memLong,dt);
  solicitada=max(0,solicitada); % Mantener la condición de no frenar.
  f=solicitada*p.drive_w;
  [f,ia]=controlABS(f,z(1:20),p);
  [f(3:4),it]=controlTraccion(f(3:4),z(1:20),p,dt);
  f(3:4)=min(f(3:4));
  h.z(k,:)=z';h.delta(k)=delta;h.rapidez(k)=estLong.vx;
  h.fuerzaPrevia(k)=solicitada;h.fuerzasSolicitadas(k,:)=f;
  h.intervencionABS(k)=any(ia.limitadas);h.intervencionTC(k)=any(it.limitadas);
  assert(all(f>=0),'No se permite solicitar frenada en esta batería.');
  try
   [zn,tele]=pasoDinamicoSoloTraccion(z,[f';delta],p,dt);
  catch errorPaso
   h.motivo=sprintf('Límite del modelo: %s',errorPaso.message);break
  end
  h.Fx(k,:)=tele.Fxw;h.Fy(k,:)=tele.Fyw;h.Fz(k,:)=tele.Fzw;
  h.capacidad(k,:)=tele.Fmax_w;h.alpha(k,:)=tele.alpha_w;h.kappa(k,:)=tele.kappa;
  z=zn;
  if z(4)<.5,h.motivo='Abortado: velocidad longitudinal inferior a 0.5 m/s tras pérdida de control';break;end
 end
 % Si un paso falla se conserva únicamente telemetría válida, sin extrapolar.
 valido=find(all(isfinite(h.Fx),2),1,'last');assert(~isempty(valido));
 campos={'t','z','delta','Fx','Fy','Fz','capacidad','alpha','kappa','rapidez','fuerzaPrevia','intervencionTC','intervencionABS','fuerzasSolicitadas'};
 for a=1:numel(campos),h.(campos{a})=h.(campos{a})(1:valido,:);end
 h.beta=atan2(h.z(:,5),h.z(:,4));
 h.uso=hypot(h.Fx,h.Fy)./max(h.capacidad,eps);
 h.lateralDisponible=sqrt(max(h.capacidad.^2-h.Fx.^2,0));
 % Indicador exploratorio, no certificación física de sobreviraje.
 tras=max(abs(h.alpha(:,3:4)),[],2);del=max(abs(h.alpha(:,1:2)),[],2);
 indicio=tras>deg2rad(8)&tras>del&abs(h.beta)>deg2rad(5);
 r=struct('caso',c.nombre,'velocidad_inicial_ms',c.velocidad,'duracion_s',h.t(end), ...
 'beta_max_grados',rad2deg(max(abs(h.beta))),'deriva_trasera_max_grados',rad2deg(max(tras)), ...
 'deriva_delantera_max_grados',rad2deg(max(del)),'uso_trasero_max',max(h.uso(:,3:4),[],'all'), ...
 'tiempo_indicio_sobreviraje_s',sum(indicio)*dt,'TC_intervenciones',sum(h.intervencionTC), ...
 'ABS_intervenciones',sum(h.intervencionABS),'motivo',h.motivo,'error_rapidez_max_ms',max(abs(h.rapidez-c.velocidad)), ...
 'error_rapidez_RMSE_ms',sqrt(mean((h.rapidez-c.velocidad).^2)), ...
 'direccion_max_grados',rad2deg(max(abs(h.delta))));
 historias{j}=h;if j==1,resumen=r;else,resumen(j)=r;end;disp(r);
end
for j=1:numel(procedencia)
 assert(strcmp(procedencia(j).sha256,hashArchivo(procedencia(j).archivo)),'Se ha modificado un bloque.');
end
save(fullfile(salida,'Ensayos_estabilidad.mat'),'historias','resumen','casos','p','procedencia','-v7.3');
writetable(struct2table(resumen),fullfile(salida,'Resumen_ensayos.csv'));
fid=fopen(fullfile(salida,'Resumen_ensayos.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(resumen,PrettyPrint=true));fclose(fid);
fid=fopen(fullfile(salida,'Integridad_bloques.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(procedencia,PrettyPrint=true));fclose(fid);
end
function [y,dy,ddy]=perfilAlce(q,L,A)
% Cambio de carril de 4 m, tramo desplazado de 10 m y retorno.
if q<0,y=0;dy=0;ddy=0;
elseif q<L,[y,dy,ddy]=transicion(q/L,L,A);
elseif q<L+10,y=A;dy=0;ddy=0;
elseif q<2*L+10,[a,b,c]=transicion((q-L-10)/L,L,A);y=A-a;dy=-b;ddy=-c;
else,y=0;dy=0;ddy=0;end
end
function [y,dy,ddy]=transicion(a,L,A)
y=A*(10*a^3-15*a^4+6*a^5);
dy=A/L*(30*a^2-60*a^3+30*a^4);
ddy=A/L^2*(60*a-180*a^2+120*a^3);
end
function valor=hashArchivo(nombre)
fid=fopen(nombre,'r');c=onCleanup(@()fclose(fid));b=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');md.update(b);valor=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
end
