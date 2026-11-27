%% Vídeo 2D sincronizado de la comparación ABS/TC
carpeta=fileparts(mfilename('fullpath'));
load(fullfile(carpeta,'lazo_completo.mat'),'sinControl','conControl','pista','p');
salida=fullfile(carpeta,'Comparacion_ABS_off_on_TC_on_2D_x4.mp4');
fps=20;aceleracion=4;duracion=max(sinControl.t(end),conControl.t(end));
tiempos=linspace(0,duracion,ceil(duracion/aceleracion*fps)+1);
casos={sinControl,conControl};colores={[.85 .18 .12],[.05 .4 .85]};
nombres={'ABS desactivado, TC activado','ABS y TC activados'};
fig=figure('Color','w','Position',[60 60 1280 720], ...
    'Visible','off','MenuBar','none','ToolBar','none','Renderer','opengl');
ax=axes(fig,'Position',[.055 .15 .90 .72]);hold(ax,'on');
plot(ax,pista.bordeIzq(:,1),pista.bordeIzq(:,2),'-','Color',[.78 .78 .78]);
plot(ax,pista.bordeDer(:,1),pista.bordeDer(:,2),'-','Color',[.78 .78 .78]);
plot(ax,pista.centro(:,1),pista.centro(:,2),'--','Color',[.65 .65 .65]);
plot(ax,pista.conoIzq(:,1),pista.conoIzq(:,2),'^','Color',[.95 .6 .1],'MarkerSize',3);
plot(ax,pista.conoDer(:,1),pista.conoDer(:,2),'^','Color',[.3 .55 .9],'MarkerSize',3);
axis(ax,'equal');xlim(ax,[-47 190]);ylim(ax,[-10 110]);grid(ax,'on');
xlabel(ax,'X [m]');ylabel(ax,'Y [m]');
title(ax,'Nuevo ABS modulado · TC, sensores y EKF activos en ambos casos','FontSize',16);
trayectorias=gobjects(1,2);coches=gobjects(1,2);frentes=gobjects(1,2);
for j=1:2
    trayectorias(j)=plot(ax,nan,nan,'Color',colores{j},'LineWidth',1.5);
    coches(j)=patch(ax,nan,nan,colores{j},'EdgeColor','k','LineWidth',1);
    frentes(j)=plot(ax,nan,nan,'w-','LineWidth',2);
end
legend(ax,trayectorias,nombres,'Location','northwest','FontSize',11);
reloj=annotation(fig,'textbox',[.05 .90 .9 .07],'EdgeColor','none', ...
    'HorizontalAlignment','center','FontSize',15,'FontWeight','bold');
indicadores=gobjects(1,2);
for j=1:2
    indicadores(j)=annotation(fig,'textbox',[.05+(j-1)*.48 .035 .47 .07], ...
        'EdgeColor','none','Color',colores{j},'FontSize',12);
end
% Rectángulos a escala visual ampliada para distinguir los dos vehículos.
contorno=[2.4 1.25;-2.4 1.25;-2.4 -1.25;2.4 -1.25]';
video=VideoWriter(salida,'MPEG-4');video.FrameRate=fps;video.Quality=90;open(video);
limpieza=onCleanup(@()cerrarVideo(video,fig));
for i=1:numel(tiempos)
    t=tiempos(i);reloj.String=sprintf('Tiempo simulado: %.1f s  |  Reproducción ×%g  |  Vehículos ampliados visualmente',t,aceleracion);
    for j=1:2
        h=casos{j};consulta=min(t,h.t(end));
        estado=interp1(h.t,h.z(:,1:6),consulta,'linear');
        psi=interp1(h.t,unwrap(h.z(:,3)),consulta,'linear');
        rotacion=[cos(psi) -sin(psi);sin(psi) cos(psi)];
        puntos=rotacion*contorno+estado(1:2)';
        set(coches(j),'XData',puntos(1,:),'YData',puntos(2,:));
        frente=rotacion*[2.1 2.1;-.9 .9]+estado(1:2)';
        set(frentes(j),'XData',frente(1,:),'YData',frente(2,:));
        idx=find(h.t<=consulta,1,'last');muestras=unique([1:10:idx idx]);
        set(trayectorias(j),'XData',h.z(muestras,1),'YData',h.z(muestras,2));
        estadoVuelta='En marcha';
        if t>=h.t(end),estadoVuelta=sprintf('Finalizado en %.2f s',h.t(end));end
        indicadores(j).String=sprintf('%s\nv = %.1f m/s · %s',nombres{j},estado(4),estadoVuelta);
    end
    drawnow;
    fotograma=getframe(fig);writeVideo(video,fotograma);
    if i==round(numel(tiempos)*.25)
        imwrite(fotograma.cdata,fullfile(carpeta,'Comparacion_ABS_off_on_TC_on_2D_x4_previa.png'));
    end
    if mod(i,200)==0,fprintf('Vídeo: %d/%d fotogramas\n',i,numel(tiempos));end
end
close(video);close(fig);clear limpieza;
lector=VideoReader(salida);
assert(lector.Width>=1280&&lector.Height>=720&&lector.Duration>=duracion/aceleracion);
fprintf('MP4 verificado: %dx%d, %.2f s, %.0f fps\n%s\n', ...
    lector.Width,lector.Height,lector.Duration,lector.FrameRate,salida);

function cerrarVideo(video,fig)
try,close(video);catch,end
if isgraphics(fig),close(fig);end
end
