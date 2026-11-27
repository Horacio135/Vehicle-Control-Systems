function Renderizar_estudio_Q_XY
% Renderiza registros existentes; no ejecuta ni modifica los bloques.
base=fileparts(mfilename('fullpath'));
valores=[.020 .015 .010 .005];verificacion=struct([]);
for caso=1:numel(valores)
 sigma=valores(caso);nombre=strrep(sprintf('Version_XY_%.3f',sigma),'.','_');
 origen=fullfile(base,nombre,'simulacion.mat');
 fprintf('\nRenderizando %d/4: %s\n',caso,nombre);
 info=renderizarCaso(origen,sigma);
 if caso==1,verificacion=info;else,verificacion(caso)=info;end
end
fid=fopen(fullfile(base,'verificacion_cuatro_videos.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(verificacion,PrettyPrint=true));fclose(fid);
fprintf('CUATRO VIDEOS VERIFICADOS\n');
end

function info=renderizarCaso(origen,sigma)
datos=load(origen,'h','pista','p');
h=datos.h;pista=datos.pista;
assert(isequal(datos.p.Q,diag([sigma sigma .005 .10 .10].^2)));
hashAntes=hashDatos(origen);
% Se conserva el vídeo también cuando el lazo pierde la trayectoria.
salida=fileparts(origen);
assert(isfolder(salida));
nombre=fullfile(salida,'Circuito_Q_XY_ABS_TC_real_estimado_2D.mp4');
fig=figure('Visible','off','Color','w','Position',[20 20 1152 720], ...
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
marcaV=xline(av,0,'k:');xlabel(av,'Tiempo [s]');ylabel(av,'vx [m/s]');
legend(av,'Real','Estimada','Location','best');
ae=nexttile(6);hold(ae,'on');grid(ae,'on');
errorPos=vecnorm(h.error_estimacion(:,1:2),2,2);
plot(ae,h.t,h.error,'Color',[.65 0 .1]);
plot(ae,h.t,h.error_estimado,'--','Color',[.45 .45 .45]);
plot(ae,h.t,errorPos,'Color',[.95 .4 .05]);
marcaE=xline(ae,0,'k:');xlabel(ae,'Tiempo [s]');ylabel(ae,'Error [m]');
legend(ae,'Seguimiento real','Seguimiento percibido','Posición EKF','Location','best','FontSize',9);
titulo=sgtitle(sprintf('Q XY = %.3f | ABS/TC habilitados | Real y estimado | x1',sigma));
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
    centro=(h.z(k,1:2)+h.estimado(k,1:2))/2;
    semiancho=max(8,abs(h.z(k,1)-h.estimado(k,1))/2+3);semialto=max(6,abs(h.z(k,2)-h.estimado(k,2))/2+3);
    xlim(ax,centro(1)+[-semiancho semiancho]);ylim(ax,centro(2)+[-semialto semialto]);
    marcaV.Value=t;marcaE.Value=t;
    titulo.String=sprintf('Q XY = %.3f | ABS/TC habilitados | Real y estimado | x1\nt = %.2f s | Error trayectoria = %.3f m | Error posición EKF = %.3f m',sigma,t,h.error(k),errorPos(k));
    drawnow;writeVideo(video,getframe(fig));
    if mod(i,100)==0,fprintf('Real/estimado: %d/%d fotogramas\n',i,numel(tiempos));end
end
clear cerrarVideo;
lector=VideoReader(nombre);
assert(lector.Duration>=h.t(end)&&lector.Width>=1000&&lector.Height>=600);
lector.CurrentTime=min(h.t(end)/2,lector.Duration-.1);
imwrite(readFrame(lector),fullfile(salida,'Circuito_Q_XY_ABS_TC_real_estimado_2D_previa.png'));
info=struct('datos_origen',origen,'caso',sprintf('Q XY %.3f, ABS/TC habilitados',sigma),'Q',datos.p.Q, ...
    'velocidad_reproduccion',1,'duracion_video_s',lector.Duration,'fps',lector.FrameRate, ...
    'tiempo_simulado_s',h.t(end),'motivo',h.motivo,'error_seguimiento_max_m',max(abs(h.error)), ...
    'rmse_posicion_EKF_m',sqrt(mean(errorPos.^2)), ...
    'definicion_error_seguimiento','Distancia lateral con signo de la posición real al circuito', ...
    'definicion_error_percibido','Distancia lateral con signo de la posición estimada al circuito', ...
    'definicion_error_estimacion','Norma de la diferencia entre posición estimada y real', ...
    'bloques_modificados',false);
fid=fopen(fullfile(salida,'Circuito_Q_XY_ABS_TC_real_estimado_2D.json'),'w','n','UTF-8');
fprintf(fid,'%s\n',jsonencode(info,PrettyPrint=true));fclose(fid);
for tiempo=[0,lector.Duration/2,max(0,lector.Duration-2/lector.FrameRate)]
 lector.CurrentTime=tiempo;assert(~isempty(readFrame(lector)));
end
assert(strcmp(hashAntes,hashDatos(origen)),'Los datos de simulación han cambiado durante el render.');
info.fotogramas_inicio_medio_final_verificados=true;
info.datos_originales_sin_cambios=true;
disp(info);disp(nombre);
end

function valor=hashDatos(nombre)
fid=fopen(nombre,'r');limpiar=onCleanup(@()fclose(fid));bytes=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');md.update(bytes);valor=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
end
