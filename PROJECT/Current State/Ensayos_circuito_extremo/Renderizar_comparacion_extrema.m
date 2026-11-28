%% Comparación simultánea: tiempo físico común, reproducción x4
base=fileparts(mfilename('fullpath'));raiz=fileparts(base);
salida=fullfile(raiz,'Resultados','Circuito_extremo_33_curvas');if ~isfolder(salida),mkdir(salida);end
datos=fullfile(salida,'Comparacion_completa.mat');if ~isfile(datos),datos=fullfile(raiz,'..','Results','Extreme_33_Corners','Perfect_State','Comparacion_completa.mat');end;d=load(datos);
colores=lines(4);fps=20;aceleracion=4;
fig=figure('Visible','off','Color','w','Position',[20 20 1440 900]);
limpieza=onCleanup(@()cerrarFigura(fig));
tl=tiledlayout(fig,2,2,'TileSpacing','compact');ax=gobjects(4,1);coche=gobjects(4,1);traza=gobjects(4,1);texto=gobjects(4,1);
for j=1:4
 ax(j)=nexttile(tl);hold(ax(j),'on');axis(ax(j),'equal');grid(ax(j),'on');
 plot(ax(j),d.pista.bordeIzq(:,1),d.pista.bordeIzq(:,2),'Color',[.7 .7 .7]);
 plot(ax(j),d.pista.bordeDer(:,1),d.pista.bordeDer(:,2),'Color',[.7 .7 .7]);
 plot(ax(j),d.pista.conoIzq(:,1),d.pista.conoIzq(:,2),'.','Color',[.95 .55 .05]);
 plot(ax(j),d.pista.conoDer(:,1),d.pista.conoDer(:,2),'.','Color',[.15 .35 .85]);
 traza(j)=plot(ax(j),nan,nan,'Color',colores(j,:),'LineWidth',1.4);
 coche(j)=patch(ax(j),nan,nan,colores(j,:),'EdgeColor','k');
 texto(j)=title(ax(j),d.nombres{j});xlabel(ax(j),'X [m]');ylabel(ax(j),'Y [m]');
end
video=VideoWriter(fullfile(salida,'Comparacion_4_coches_2D_x4.mp4'),'MPEG-4');video.FrameRate=fps;video.Quality=90;open(video);
cierre=onCleanup(@()close(video));
fin=max(cellfun(@(h)h.t(end),d.historias));tiempos=unique([0:aceleracion/fps:fin fin]);
for n=1:numel(tiempos)
 t=tiempos(n);
 for j=1:4
  h=d.historias{j};k=min(numel(h.t),floor(t/d.p.dt)+1);z=h.z(k,:);
  cuerpo=[1.1 .65;1.1 -.65;-1 -.65;-1 .65];R=[cos(z(3)) -sin(z(3));sin(z(3)) cos(z(3))];xy=cuerpo*R'+z(1:2);
  set(coche(j),'XData',xy(:,1),'YData',xy(:,2));
  ids=max(1,k-10000):25:k;set(traza(j),'XData',h.z(ids,1),'YData',h.z(ids,2));
  xlim(ax(j),z(1)+[-42 42]);ylim(ax(j),z(2)+[-25 25]);
  estado=sprintf('ABS %d | TC %d | ESC %d',h.ABS(k),h.TC(k),h.ESC(k));
  if t>h.t(end),estado=['FINAL: ' h.motivo ' (imagen retenida)'];end
  set(texto(j),'String',{sprintf('%s | %.1f m/s | avance %.1f%%',d.nombres{j},z(4),100*h.avance(k)/d.pista.longitud),estado},'FontSize',10);
 end
 title(tl,sprintf('Circuito extremo: 33 curvas | t = %.1f s | reproducción x4',t));drawnow;
 writeVideo(video,getframe(fig));
end
clear cierre;
% Capacidad y utilización real por rueda; el círculo no es una medida de deslizamiento.
fig2=figure('Visible','off','Color','w','Position',[20 20 1500 950]);tiledlayout(fig2,4,2);
for j=1:4
 h=d.historias{j};ids=1:10:numel(h.t);
 nexttile;plot(h.t(ids),h.capacidad(ids,:));grid on;title(d.nombres{j});ylabel('Capacidad [N]');xlabel('Tiempo [s]');legend('DI','DD','TI','TD');
 nexttile;plot(h.t(ids),hypot(h.Fx(ids,:),h.Fy(ids,:))./max(h.capacidad(ids,:),eps));grid on;ylabel('Utilización combinada');xlabel('Tiempo [s]');ylim([0 1.2]);
end
exportgraphics(fig2,fullfile(salida,'Capacidad_y_utilizacion_por_rueda.png'),'Resolution',150);close(fig2);
disp('Vídeo y gráficas generados.');

function cerrarFigura(f)
if isgraphics(f),close(f);end
end
