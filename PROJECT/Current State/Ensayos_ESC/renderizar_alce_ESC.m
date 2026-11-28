function renderizar_alce_ESC
% Comparación sincronizada; el caso abortado se congela, no se extrapola.
base=fileparts(fileparts(mfilename('fullpath')));salida=fullfile(base,'Resultados','Alce_02_ESC_V1_transiciones_13m_2026-11-28');
d=load(fullfile(salida,'Ensayos_estabilidad.mat'),'historias');hs=d.historias;
colores=[.85 .15 .12;0 .4 .85];nombres={'Sin ESC','Con ESC'};
fig=figure('Visible','off','Color','w','Position',[20 20 1280 720],'MenuBar','none','ToolBar','none');
limpieza=onCleanup(@()close(fig));tiledlayout(3,3,'Padding','compact','TileSpacing','compact');
ax=nexttile([3 2]);hold(ax,'on');axis(ax,'equal');grid(ax,'on');
ref=hs{1}.ref;plot(ax,ref(:,1),ref(:,2),'k--','HandleVisibility','off');
plot(ax,ref(:,1),ref(:,2)+1.5,':','Color',[.6 .6 .6],'HandleVisibility','off');
plot(ax,ref(:,1),ref(:,2)-1.5,':','Color',[.6 .6 .6],'HandleVisibility','off');
ids=1:40:size(ref,1);plot(ax,ref(ids,1),ref(ids,2)+1.5,'^','Color',[.95 .6 .1],'HandleVisibility','off');
plot(ax,ref(ids,1),ref(ids,2)-1.5,'^','Color',[.2 .5 .9],'HandleVisibility','off');
xo=40+hs{1}.config.longitud+5;
rectangle(ax,'Position',[xo-2 -1 4 2],'FaceColor',[.5 .5 .5]);text(ax,xo,0,'Obstáculo','Color','w','HorizontalAlignment','center','FontSize',8);
coches=gobjects(1,2);frente=coches;trazas=coches;
for j=1:2
 trazas(j)=plot(ax,nan,nan,'Color',colores(j,:),'LineWidth',1.8);
 coches(j)=patch(ax,nan,nan,colores(j,:),'FaceAlpha',.65,'EdgeColor',colores(j,:),'LineWidth',1.5);
 frente(j)=plot(ax,nan,nan,'w-','LineWidth',2,'HandleVisibility','off');
end
legend(ax,trazas,nombres,'Location','northwest');xlabel(ax,'X [m]');ylabel(ax,'Y [m]');
texto=text(ax,.02,.02,'','Units','normalized','VerticalAlignment','bottom','FontSize',10,'BackgroundColor','w','Margin',4);
marcas=gobjects(1,3);
for j=1:3
 a=nexttile(3*j);hold(a,'on');grid(a,'on');
 for caso=1:2
  h=hs{caso};if j==1,y=rad2deg(h.beta);elseif j==2,y=h.z(:,6);else,y=h.rapidez;end
  plot(a,h.t,y,'Color',colores(caso,:));
 end
 if j==1,ylabel(a,'Beta [grados]');elseif j==2,plot(a,hs{2}.t,hs{2}.rRef,'k--');ylabel(a,'Guiñada [rad/s]');else,ylabel(a,'Rapidez [m/s]');end
 marcas(j)=xline(a,0,'k:','HandleVisibility','off');xlabel(a,'Tiempo [s]');xlim(a,[0 hs{2}.t(end)]);
end
encabezado=sgtitle('Alce nº2 | ESC V1 | transiciones13m | Estado perfecto | ABS y TC activos | ESC solo frenada, sin corrección de dirección');
nombre=fullfile(salida,'Comparacion_alce_02_sin_con_ESC_2D.mp4');
v=VideoWriter(nombre,'MPEG-4');v.FrameRate=20;v.Quality=90;open(v);cerrar=onCleanup(@()close(v));
tiempos=unique([0:1/v.FrameRate:hs{2}.t(end),hs{2}.t(end)]);vertices=[1 .6;1 -.6;-1 -.6;-1 .6];
for k=1:numel(tiempos)
 t=tiempos(k);posiciones=zeros(2,2);indicadores=cell(1,2);
 for j=1:2
  h=hs{j};consulta=min(t,h.t(end));idx=find(h.t<=consulta,1,'last');estado=h.z(idx,1:3);posiciones(j,:)=estado(1:2);
  rot=[cos(estado(3)) sin(estado(3));-sin(estado(3)) cos(estado(3))];
  vs=vertices*rot+estado(1:2);set(coches(j),'XData',vs(:,1),'YData',vs(:,2));
  vs=[.8 -.45;.8 .45]*rot+estado(1:2);set(frente(j),'XData',vs(:,1),'YData',vs(:,2));
  muestras=unique([1:5:idx idx]);set(trazas(j),'XData',h.z(muestras,1),'YData',h.z(muestras,2));
  estadoTexto='En marcha';if t>h.t(end),estadoTexto=[h.motivo ' (imagen congelada)'];end
  if j==2 && h.ESC(idx),estadoTexto='ESC: motor retirado + freno selectivo';end
  indicadores{j}=sprintf('%s: v=%.1f m/s | beta=%.1f° | %s',nombres{j},h.rapidez(idx),rad2deg(h.beta(idx)),estadoTexto);
 end
 xlim(ax,[min(posiciones(:,1))-12 max(posiciones(:,1))+18]);ylim(ax,[min(-3,min(posiciones(:,2)))-6 max(7,max(posiciones(:,2)))+6]);
 texto.String=sprintf('%s\n%s',indicadores{:});
 for j=1:3,marcas(j).Value=t;end
 encabezado.String=sprintf('Alce nº2 | ESC V1 | transiciones13m | t=%.2f s | x1 | ESC: frenada selectiva, sin corrección de dirección',t);
 drawnow;imagen=getframe(fig);writeVideo(v,imagen);
 if abs(t-3)<.01,imwrite(imagen.cdata,fullfile(salida,'Comparacion_alce_ESC_previa.png'));end
end
clear cerrar;lector=VideoReader(nombre);
for f=[0 .5 .99],lector.CurrentTime=f*(lector.Duration-1/lector.FrameRate);assert(~isempty(readFrame(lector)));end
fprintf('Vídeo simultáneo verificado: %dx%d, %.2fs, x1\n%s\n',lector.Width,lector.Height,lector.Duration,nombre);
end
