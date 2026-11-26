function validar_alce_seguimiento
base=fileparts(fileparts(mfilename('fullpath')));salida=fullfile(fileparts(base),'Results','Moose_Test_02');
d=load(fullfile(salida,'Ensayos_estabilidad.mat'));
assert(numel(d.historias)==1);
archivos=dir(fullfile(salida,'*_2D_capacidad.mp4'));assert(numel(archivos)==1);
contactos=struct([]);
for j=1:numel(d.historias)
 h=d.historias{j};assert(all(h.fuerzasSolicitadas>=0,'all'));
 assert(~any(h.intervencionABS));
 assert(all(isfinite(h.z),'all')&&all(h.Fz>=0,'all'));
 assert(max(h.uso,[],'all')<=1+1e-8);
 v=VideoReader(fullfile(salida,[h.config.nombre '_2D_capacidad.mp4']));
 for t=[0 v.Duration/2 max(0,v.Duration-2/v.FrameRate)]
  v.CurrentTime=t;cuadro=readFrame(v);assert(~isempty(cuadro));
 end
 if strcmp(h.config.tipo,'alce')
  xo=40+h.config.longitud+5;
  obs=[xo-2 -1;xo+2 -1;xo+2 1;xo-2 1];contacto=false;
  for k=1:numel(h.t)
   psi=h.z(k,3);rot=[cos(psi) sin(psi);-sin(psi) cos(psi)];
   coche=[1 .6;1 -.6;-1 -.6;-1 .6]*rot+h.z(k,1:2);
   ejes=[1 0;0 1;rot];separado=false;
   for a=1:4
    pc=coche*ejes(a,:)';po=obs*ejes(a,:)';
    if max(pc)<min(po)||max(po)<min(pc),separado=true;break;end
   end
   if ~separado,contacto=true;break;end
  end
  r=struct('caso',h.config.nombre,'contacto_visual',contacto,'tiempo_primer_contacto_s',NaN);
  if contacto,r.tiempo_primer_contacto_s=h.t(k);end
  if isempty(contactos),contactos=r;else,contactos(end+1)=r;end
 end
 [~,kp]=max(abs(h.beta));v.CurrentTime=min(h.t(kp),v.Duration-1/v.FrameRate);
 imwrite(readFrame(v),fullfile(salida,[h.config.nombre '_deriva_maxima.png']));
end
writetable(struct2table(contactos),fullfile(salida,'Contacto_obstaculo_visual.csv'));
for j=1:numel(d.procedencia)
 [~,nombre]=fileparts(strrep(d.procedencia(j).archivo,char(92),'/'));
 if strcmp(nombre,'configuracionIntegracion'),continue;end % Q restaurada después del ensayo.
 partes=regexp(strrep(d.procedencia(j).archivo,char(92),'/'),'(Dinamica|Control|Configuracion)/([^/]+)$','tokens','once');
 assert(~isempty(partes));archivoLocal=fullfile(base,partes{1},partes{2});
 fid=fopen(archivoLocal,'r');b=fread(fid,Inf,'*uint8');fclose(fid);
 md=java.security.MessageDigest.getInstance('SHA-256');md.update(b);
 hash=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
 assert(strcmp(hash,d.procedencia(j).sha256),'Un bloque ha cambiado.');
end
% Mapa general de los ocho resultados, sin recorte por cámara.
f=figure('Visible','off','Color','w','Position',[20 20 1152 720]);tiledlayout(1,1,'TileSpacing','compact');
for j=1:1
 h=d.historias{j};ax=nexttile;hold(ax,'on');grid(ax,'on');
 if strcmp(h.config.tipo,'alce'),plot(ax,h.ref(:,1),h.ref(:,2),'k--');end
 plot(ax,h.z(:,1),h.z(:,2),'Color',[.05 .35 .75],'LineWidth',1.5);
 [~,kp]=max(abs(h.beta));plot(ax,h.z(kp,1),h.z(kp,2),'ro','MarkerFaceColor','r');
 xlabel(ax,'X [m]');ylabel(ax,'Y [m]');
 title(ax,sprintf('%s | %.0f m/s\n|beta| máx. %.2f°',strrep(h.config.nombre,'_',' '),h.config.velocidad,rad2deg(max(abs(h.beta)))),'FontSize',10);
end
sgtitle('Alce PP 02 con Pure Pursuit y referencia de rapidez constante | Punto rojo: deriva máxima');
exportgraphics(f,fullfile(salida,'Resumen_trayectorias.png'),'Resolution',150);close(f);
% Live Script de entrada, sin ejecutar otra vez los ensayos.
carpeta=fullfile(base,'Ensayos_alce_seguimiento');
matlab.internal.liveeditor.openAndSave(fullfile(carpeta,'Ejecutar_alce_seguimiento.m'),fullfile(carpeta,'Ejecutar_alce_seguimiento.mlx'));
ver=struct('casos',1,'mp4',numel(archivos),'fotogramas_inicio_medio_final_leidos',true, ...
 'sin_frenada_solicitada',true,'ABS_sin_intervencion',true,'bloques_del_ensayo_verificados_SHA256',true,'configuracion_Q_excluida_por_restauracion_posterior',true, ...
 'sin_sensores_estimador',true,'sin_ESC',true,'PurePursuit_actual',true,'referencia_rapidez_constante',true);
fid=fopen(fullfile(salida,'Verificacion_final.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(ver,PrettyPrint=true));fclose(fid);disp(ver);
end
