function renderizar_nuevo_estimado
carpetaResultados=fileparts(mfilename('fullpath'));
origen=fullfile(carpetaResultados,'lazo_completo.mat');
datos=load(origen,'conControl','pista','resumen');
h=datos.conControl;pista=datos.pista;
assert(strcmp(h.motivo,'Vuelta completada'));
salida=fileparts(origen);
assert(isfolder(salida));
nombre=fullfile(salida,'ABS_TC_on_real_estimado_2D.mp4');
fig=figure('Visible','off','Color','w','Position',[40 40 1280 720], ...
    'MenuBar','none','ToolBar','none','Renderer','opengl');
limpieza=onCleanup(@()close(fig));
tiledlayout(2,3,'TileSpacing','compact','Padding','compact');
ax=nexttile([2 2]);hold(ax,'on');grid(ax,'on');axis(ax,'equal');
plot(ax,pista.centro(:,1),pista.centro(:,2),'k--','LineWidth',1);
plot(ax,pista.bordeIzq(:,1),pista.bordeIzq(:,2),'Color',[.65 .65 .65]);
plot(ax,pista.bordeDer(:,1),pista.bordeDer(:,2),'Color',[.65 .65 .65]);
plot(ax,pista.conoIzq(:,1),pista.conoIzq(:,2),'^','Color',[.9 .65 .05],'MarkerSize',4);
plot(ax,pista.conoDer(:,1),pista.conoDer(:,2),'^','Color',[.35 .6 .9],'MarkerSize',4);
trReal=plot(ax,NaN,NaN,'Color',[0 .35 .8],'LineWidth',2);
trEst=plot(ax,NaN,NaN,'--','Color',[.95 .4 .05],'LineWidth',1.6);
real=patch(ax,NaN,NaN,[0 .35 .8],'FaceAlpha',.55,'EdgeColor',[0 .2 .5]);
estimado=patch(ax,NaN,NaN,[.95 .4 .05],'FaceColor','none','EdgeColor',[.95 .4 .05],'LineStyle','--','LineWidth',1.5);
frenteReal=plot(ax,NaN,NaN,'w-','LineWidth',2);
frenteEst=plot(ax,NaN,NaN,'Color',[.95 .4 .05],'LineWidth',1.5);
legend(ax,[trReal trEst],{'Trayectoria real','Trayectoria estimada'},'Location','northwest');
xlabel(ax,'X [m]');ylabel(ax,'Y [m]');
av=nexttile(3);hold(av,'on');grid(av,'on');
plot(av,h.t,h.z(:,4),'Color',[0 .35 .8]);plot(av,h.t,h.estimado(:,4),'--','Color',[.95 .4 .05]);
marcaV=xline(av,0,'k:','HandleVisibility','off');xlabel(av,'Tiempo [s]');ylabel(av,'vx [m/s]');
legend(av,'Real','Estimada','Location','best');
ae=nexttile(6);hold(ae,'on');grid(ae,'on');
errorPos=vecnorm(h.error_estimacion(:,1:2),2,2);
plot(ae,h.t,h.error,'Color',[.65 0 .1]);
plot(ae,h.t,h.error_estimado,'--','Color',[.45 .45 .45]);
plot(ae,h.t,errorPos,'Color',[.95 .4 .05]);
marcaE=xline(ae,0,'k:','HandleVisibility','off');xlabel(ae,'Tiempo [s]');ylabel(ae,'Error [m]');
legend(ae,'Seguimiento real','Seguimiento percibido','Posición EKF','Location','best','FontSize',9);
titulo=sgtitle('Lazo actual con ABS/TC | Real y estimado | Velocidad normal');
video=VideoWriter(nombre,'MPEG-4');video.FrameRate=20;video.Quality=90;open(video);
cerrarVideo=onCleanup(@()close(video));
tiempos=unique([0:1/video.FrameRate:h.t(end),h.t(end)]);
vertices=[1 .58;1 -.58;-1 -.58;-1 .58];
for i=1:numel(tiempos)
    t=tiempos(i);k=find(h.t<=t,1,'last');muestras=unique([1:10:k k]);
    set(trReal,'XData',h.z(muestras,1),'YData',h.z(muestras,2));
    set(trEst,'XData',h.estimado(muestras,1),'YData',h.estimado(muestras,2));
    for j=1:2
        if j==1,estado=h.z(k,1:3);objeto=real;frente=frenteReal;
        else,estado=h.estimado(k,1:3);objeto=estimado;frente=frenteEst;end
        c=cos(estado(3));s=sin(estado(3));rotacion=[c s;-s c];
        v=vertices*rotacion+estado(1:2);set(objeto,'XData',v(:,1),'YData',v(:,2));
        v=[.8 -.42;.8 .42]*rotacion+estado(1:2);set(frente,'XData',v(:,1),'YData',v(:,2));
    end
    xlim(ax,h.z(k,1)+[-8 8]);ylim(ax,h.z(k,2)+[-6 6]);
    marcaV.Value=t;marcaE.Value=t;
    titulo.String=sprintf('ABS/TC activos | t = %.2f s | Seguimiento = %.3f m | Error EKF = %.3f m | x1',t,h.error(k),errorPos(k));
    drawnow;writeVideo(video,getframe(fig));
    if mod(i,200)==0,fprintf('Real/estimado: %d/%d fotogramas\n',i,numel(tiempos));end
end
clear cerrarVideo;
lector=VideoReader(nombre);
assert(lector.Duration>=h.t(end)&&lector.Width>=1000&&lector.Height>=600);
lector.CurrentTime=min(15,lector.Duration-.1);
imwrite(readFrame(lector),fullfile(salida,'ABS_TC_on_real_estimado_2D_previa.png'));
info=struct('datos_origen',origen,'caso','Lazo completo con ABS/TC después del ajuste', ...
    'velocidad_reproduccion',1,'duracion_video_s',lector.Duration,'fps',lector.FrameRate, ...
    'tiempo_vuelta_s',h.t(end),'error_seguimiento_max_m',max(abs(h.error)), ...
    'rmse_posicion_EKF_m',sqrt(mean(errorPos.^2)), ...
    'definicion_error_seguimiento','Distancia lateral con signo de la posición real al circuito', ...
    'definicion_error_percibido','Distancia lateral con signo de la posición estimada al circuito', ...
    'definicion_error_estimacion','Norma de la diferencia entre posición estimada y real', ...
    'bloques_modificados',{{'controlABS','controlTotal','configuracion'}});
fid=fopen(fullfile(salida,'ABS_TC_on_real_estimado_2D.json'),'w','n','UTF-8');
fprintf(fid,'%s\n',jsonencode(info,PrettyPrint=true));fclose(fid);
disp(info);disp(nombre);
end
