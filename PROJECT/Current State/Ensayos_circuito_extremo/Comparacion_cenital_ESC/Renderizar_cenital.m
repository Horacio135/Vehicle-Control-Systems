%% Circuito completo fijo y dos coches, tiempo físico común
base=fileparts(mfilename('fullpath'));raiz=fileparts(fileparts(base));addpath(fileparts(base),raiz,'-begin');salida=fullfile(raiz,'Resultados','Comparacion_ESC_recuperacion');if ~isfolder(salida),mkdir(salida);end
datos=fullfile(salida,'Comparacion_completa.mat');if isfile(datos),d=load(datos);else,publicados=fullfile(raiz,'..','Results','Extreme_33_Corners','ESC_Recovery');d=load(fullfile(publicados,'Contexto_comparacion.mat'));a=load(fullfile(publicados,'Caso_1.mat'),'h');b=load(fullfile(publicados,'Caso_2.mat'),'h');d.historias={a.h,b.h};end;colores=[.95 .4 .05;.05 .35 .9];
fig=figure('Visible','off','Color','w','Position',[20 20 1800 1150]);
ax=axes(fig,'Position',[.06 .13 .9 .79]);hold(ax,'on');axis(ax,'equal');grid(ax,'on');
plot(d.pista.bordeIzq(:,1),d.pista.bordeIzq(:,2),'Color',[.6 .6 .6]);
plot(d.pista.bordeDer(:,1),d.pista.bordeDer(:,2),'Color',[.6 .6 .6]);
todos=[d.pista.centro;d.historias{1}.z(:,1:2);d.historias{2}.z(:,1:2)];xlim([min(todos(:,1))-35 max(todos(:,1))+35]);
ylim([min(todos(:,2))-35 max(todos(:,2))+35]);
plot(d.pista.centro(1,1),d.pista.centro(1,2),'ks','MarkerFaceColor','w','MarkerSize',7);text(d.pista.centro(1,1),d.pista.centro(1,2)-20,'Salida / meta','FontSize',10);xlabel('X [m]');ylabel('Y [m]');traza=gobjects(2,1);marca=gobjects(2,1);flecha=gobjects(2,1);
for j=1:2
 traza(j)=plot(nan,nan,'Color',colores(j,:),'LineWidth',1.4);
 marca(j)=plot(nan,nan,'o','MarkerSize',14-3*j,'MarkerFaceColor',colores(j,:),'MarkerEdgeColor','k');
 flecha(j)=quiver(0,0,0,0,0,'Color',colores(j,:),'LineWidth',2,'MaxHeadSize',1);
end
legend(marca,d.nombres,'Location','southoutside','Orientation','horizontal');
hud=annotation(fig,'textbox',[.05 .01 .9 .09],'EdgeColor','none','FontSize',12);
video=VideoWriter(fullfile(salida,'Cenital_completa_ABS_TC_con_y_sin_ESC_x8.mp4'),'MPEG-4');video.FrameRate=20;video.Quality=95;open(video);
fin=max(cellfun(@(h)h.t(end),d.historias));tiempos=unique([0:.4:fin fin]);
try
 for n=1:numel(tiempos)
  t=tiempos(n);lineas=cell(1,2);
  for j=1:2
   h=d.historias{j};k=min(numel(h.t),floor(t/d.p.dt)+1);z=h.z(k,:);ids=1:30:k;
   set(traza(j),'XData',h.z(ids,1),'YData',h.z(ids,2));set(marca(j),'XData',z(1),'YData',z(2));
   set(flecha(j),'XData',z(1),'YData',z(2),'UData',18*cos(z(3)),'VData',18*sin(z(3)));
   estado='conducción original';if h.vRef(k)<=2,estado='recuperación temporal del conductor';end;if t>=h.t(end),estado=h.motivo;end
   lineas{j}=sprintf('%s: %.1f m/s | %.1f%% | %s',d.nombres{j},z(4),100*h.avance(k)/d.pista.longitud,estado);
  end
  title(sprintf('Vista cenital fija del circuito completo | t = %.1f s | x8',t));
  set(hud,'String',[lineas {'Símbolos de los coches ampliados. Recuperación del conductor sin reinicio físico; sensores/EKF omitidos.'}]);
  drawnow;writeVideo(video,getframe(fig));if mod(n,200)==0,fprintf('Render cenital: %d/%d fotogramas\n',n,numel(tiempos));end
  if n==30,exportgraphics(fig,fullfile(salida,'Vista_cenital.png'),'Resolution',120);end
 end
 close(video);close(fig);
catch fallo
 close(video);if isgraphics(fig),close(fig);end;rethrow(fallo);
end
disp('Render cenital terminado.');
