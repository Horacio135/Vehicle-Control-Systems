function renderizar_alce_seguimiento
% Vídeo 2D del único caso conservado: Alce PP 02.
base=fileparts(fileparts(mfilename('fullpath')));
salida=fullfile(fileparts(base),'Results','Moose_Test_02');
datos=load(fullfile(salida,'Ensayos_estabilidad.mat'));fps=15;verificacion=struct([]);
for j=1:numel(datos.historias)
 h=datos.historias{j};c=h.config;
 fig=figure('Visible','off','Color','w','Position',[20 20 1152 720],'MenuBar','none','ToolBar','none');
 limpiar=onCleanup(@()close(fig));
 tiledlayout(fig,5,3,'TileSpacing','compact','Padding','compact');
 ax=nexttile(1,[5 2]);hold(ax,'on');grid(ax,'on');axis(ax,'equal');
 if strcmp(c.tipo,'alce')
  plot(ax,h.ref(:,1),h.ref(:,2),'k--','LineWidth',1.2,'DisplayName','Guía geométrica de esquiva');
  plot(ax,h.ref(:,1),h.ref(:,2)+1.5,':','Color',[.6 .6 .6],'HandleVisibility','off');
  plot(ax,h.ref(:,1),h.ref(:,2)-1.5,':','Color',[.6 .6 .6],'HandleVisibility','off');
  ids=1:40:size(h.ref,1);
  plot(ax,h.ref(ids,1),h.ref(ids,2)+1.5,'^','Color',[.95 .6 0],'HandleVisibility','off');
  plot(ax,h.ref(ids,1),h.ref(ids,2)-1.5,'^','Color',[.1 .5 .9],'HandleVisibility','off');
  xo=40+c.longitud+5;
  rectangle(ax,'Position',[xo-2 -1 4 2],'FaceColor',[.7 .15 .12],'EdgeColor','none');
  text(ax,xo,0,'OBSTÁCULO','Color','w','HorizontalAlignment','center','FontSize',8);
 else
  text(ax,.02,.98,sprintf('Dirección: amplitud %.1f° | frecuencia %.2f Hz\nCinco ciclos con entrada y salida progresivas',rad2deg(c.amplitud),c.frecuencia), ...
   'Units','normalized','VerticalAlignment','top','FontSize',11);
 end
 plot(ax,h.z(:,1),h.z(:,2),'Color',[.82 .86 .91],'HandleVisibility','off');
 recorrido=plot(ax,NaN,NaN,'Color',[.08 .35 .75],'LineWidth',2,'DisplayName','Trayectoria real');
 coche=patch(ax,NaN,NaN,[.08 .35 .75],'FaceAlpha',.8,'HandleVisibility','off');
 frente=plot(ax,NaN,NaN,'w-','LineWidth',2,'HandleVisibility','off');
 rumbo=plot(ax,NaN,NaN,'Color',[.15 .15 .15],'LineWidth',1.5,'DisplayName','Orientación del vehículo');
 vectorV=plot(ax,NaN,NaN,'Color',[.1 .6 .3],'LineWidth',1.5,'DisplayName','Dirección de velocidad');
 texto=text(ax,.02,.02,'','Units','normalized','VerticalAlignment','bottom','FontSize',10,'BackgroundColor','w','Margin',5);
 xlabel(ax,'X [m]');ylabel(ax,'Y [m]');legend(ax,'Location','southoutside');
 xlim(ax,[min([h.z(:,1);h.ref(:,1)])-6 max([h.z(:,1);h.ref(:,1)])+6]);
 yy=[h.z(:,2);h.ref(:,2)];ylim(ax,[min(yy)-8 max(yy)+8]);
 title(ax,sprintf('%s | v referencia %.0f m/s | Pure Pursuit + control de velocidad',strrep(c.nombre,'_',' '),c.velocidad));
 av=nexttile(15);hold(av,'on');grid(av,'on');plot(av,h.t,h.rapidez,'Color',[.05 .35 .75]);yline(av,c.velocidad,'k--');mv=xline(av,0,'r:');xlim(av,[0 h.t(end)]);ylim(av,[0 c.velocidad*1.1]);ylabel(av,'Rapidez [m/s]');xlabel(av,'Tiempo [s]');
 marcas=gobjects(1,4);ruedas={'Delantera izquierda','Delantera derecha','Trasera izquierda','Trasera derecha'};
 for w=1:4
  ag=nexttile(3*w);hold(ag,'on');grid(ag,'on');
  plot(ag,h.t,h.capacidad(:,w),'k','LineWidth',1.2);
  disponibleX=sqrt(max(h.capacidad(:,w).^2-h.Fy(:,w).^2,0));
  plot(ag,h.t,disponibleX,'--','Color',[.9 .45 .05],'LineWidth',1);
  plot(ag,h.t,abs(h.Fx(:,w)),'Color',[.1 .6 .3]);
  plot(ag,h.t,abs(h.Fy(:,w)),'Color',[.1 .35 .85]);
  marcas(w)=xline(ag,0,'r:');
  xlim(ag,[0 h.t(end)]);ylim(ag,[0 max(h.capacidad,[],'all')*1.07]);
  ylabel(ag,'Fuerza [N]');title(ag,ruedas{w});
  if w==1,legend(ag,'Capacidad total','Margen longitudinal geom.','|Fx| real','|Fy| real','Location','best','FontSize',7);end
  if w==4,xlabel(ag,'Tiempo [s]');end
 end
 titulo=sgtitle(sprintf('%s | ABS/TC habilitados | Sin sensores, estimador ni ESC',strrep(c.nombre,'_',' ')));
 nombre=fullfile(salida,[c.nombre '_2D_capacidad.mp4']);
 v=VideoWriter(nombre,'MPEG-4');v.FrameRate=fps;v.Quality=88;open(v);cerrar=onCleanup(@()close(v));
 tiempos=unique([0:1/fps:h.t(end),h.t(end)]);
 vertices=[1 .6;1 -.6;-1 -.6;-1 .6];
 for a=1:numel(tiempos)
  tiempo=tiempos(a);k=find(h.t<=tiempo,1,'last');pos=h.z(k,1:2);psi=h.z(k,3);
  rot=[cos(psi) sin(psi);-sin(psi) cos(psi)];vs=vertices*rot+pos;
  set(coche,'XData',vs(:,1),'YData',vs(:,2));
  xlim(ax,pos(1)+[-20 23]);ylim(ax,pos(2)+[-15 15]);
  if abs(h.beta(k))>deg2rad(5),coche.FaceColor=[.85 .2 .08];else,coche.FaceColor=[.08 .35 .75];end
  vv=[.8 -.45;.8 .45]*rot+pos;set(frente,'XData',vv(:,1),'YData',vv(:,2));
  set(recorrido,'XData',h.z(1:k,1),'YData',h.z(1:k,2));
  set(rumbo,'XData',[pos(1) pos(1)+4*cos(psi)],'YData',[pos(2) pos(2)+4*sin(psi)]);
  ang=psi+h.beta(k);set(vectorV,'XData',[pos(1) pos(1)+4*cos(ang)],'YData',[pos(2) pos(2)+4*sin(ang)]);
  texto.String=sprintf('t %.2f s | v %.2f m/s | dirección %.2f°\nDeriva vehículo β %.2f° | guiñada %.2f°/s\nDerivas traseras %.2f° / %.2f° | uso trasero %.1f / %.1f %%\nTC limitado: %d | ABS habilitado, sin frenada | Sin ESC', ...
   h.t(k),hypot(h.z(k,4),h.z(k,5)),rad2deg(h.delta(k)),rad2deg(h.beta(k)),rad2deg(h.z(k,6)), ...
   rad2deg(h.alpha(k,3)),rad2deg(h.alpha(k,4)),100*h.uso(k,3),100*h.uso(k,4),h.intervencionTC(k));
  for w=1:4,marcas(w).Value=tiempo;end;mv.Value=tiempo;
  titulo.String=sprintf('%s | %.2f s | v real %.2f / referencia %.0f m/s | Pure Pursuit | x1',strrep(c.nombre,'_',' '),tiempo,h.rapidez(k),c.velocidad);
  drawnow;writeVideo(v,getframe(fig));
  if mod(a,100)==0,fprintf('%s: %d/%d fotogramas\n',c.nombre,a,numel(tiempos));end
 end
 clear cerrar;
 lector=VideoReader(nombre);lector.CurrentTime=min(h.t(end)/2,lector.Duration-.1);imwrite(readFrame(lector),fullfile(salida,[c.nombre '_previa.png']));
 assert(lector.Width>=1200 && lector.Height>=700 && lector.Duration>=h.t(end));
 ver=struct('archivo',nombre,'ancho',lector.Width,'alto',lector.Height,'duracion_s',lector.Duration,'fps',lector.FrameRate,'fotograma_decodificado',true);
 if j==1,verificacion=ver;else,verificacion(j)=ver;end
 % Figura completa de diagnóstico, independiente del vídeo.
 f=figure('Visible','off','Color','w','Position',[20 20 1152 720]);tiledlayout(3,2,'TileSpacing','compact');
 for w=1:4
  ag=nexttile;hold(ag,'on');grid(ag,'on');
  plot(ag,h.t,h.capacidad(:,w),'k');plot(ag,h.t,abs(h.Fx(:,w)),'Color',[.1 .6 .3]);plot(ag,h.t,abs(h.Fy(:,w)),'b');plot(ag,h.t,sqrt(max(h.capacidad(:,w).^2-h.Fy(:,w).^2,0)),'--','Color',[.65 .3 .1]);
  title(ag,ruedas{w});ylabel(ag,'N');xlabel(ag,'Tiempo [s]');legend(ag,'Capacidad total','|Fx|','|Fy|','Margen longitudinal geom.','Location','best');
 end
 ag=nexttile;plot(ag,h.t,rad2deg(h.beta),'k',h.t,rad2deg(h.alpha(:,3:4)),'LineWidth',1);grid(ag,'on');ylabel(ag,'Ángulo [°]');xlabel(ag,'Tiempo [s]');legend(ag,'Beta vehículo','Deriva TI','Deriva TD');
 ag=nexttile;yyaxis(ag,'left');plot(ag,h.t,rad2deg(h.delta));ylabel(ag,'Dirección [°]');yyaxis(ag,'right');plot(ag,h.t,h.rapidez);hold(ag,'on');yline(ag,c.velocidad,'k--');ylabel(ag,'Rapidez / referencia [m/s]');xlabel(ag,'Tiempo [s]');grid(ag,'on');
 sgtitle(sprintf('%s | Fuerzas, capacidad y deriva | %s',strrep(c.nombre,'_',' '),h.motivo));
 exportgraphics(f,fullfile(salida,[c.nombre '_graficas.png']),'Resolution',150);savefig(f,fullfile(salida,[c.nombre '_graficas.fig']));close(f);
 clear limpiar;
 fprintf('Vídeo %d/1 verificado: %s\n',j,nombre);
end
fid=fopen(fullfile(salida,'Verificacion_videos.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(verificacion,PrettyPrint=true));fclose(fid);
end
