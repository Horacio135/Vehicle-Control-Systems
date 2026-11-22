function fig = Animar_planificador_hibrido(resultados,pista,opciones)
% Reproduce historicos ya calculados; no integra ni modifica la fisica.
% Uso: Animar_planificador_hibrido(resultados,pista)
% Video: opciones.video='seguimiento.mp4'; opciones.reproducir=false;
% El video se renderiza desde la figura de MATLAB, sin capturar el escritorio.
if nargin<3, opciones=struct(); end
if ~isfield(opciones,'fps'), opciones.fps=20; end
if ~isfield(opciones,'reproducir'), opciones.reproducir=true; end
if ~isfield(opciones,'video'), opciones.video=''; end
validateattributes(opciones.fps,{'numeric'},{'scalar','positive','finite'});
n=numel(resultados); assert(n>0,'No hay historicos para animar.');
T=max(cellfun(@(h) h.t(end),resultados));
tiempos=(0:1/opciones.fps:T)';
if tiempos(end)<T, tiempos(end+1)=T; end
colores=lines(n);
datos=cell(n,1);
for i=1:n
    h=resultados{i}; consulta=min(tiempos,h.t(end));
    datos{i}=interp1(h.t,[h.z(:,1:2),unwrap(h.z(:,3)),h.z(:,4:5),h.vref],consulta,'linear');
end
exportando=~isempty(opciones.video);
fig=figure('Name','Animacion - planificador V9 y Pure Pursuit','Color','w', ...
    'Position',[100 80 1200 820],'NumberTitle','off');
if exportando, fig.Visible='off'; end
ax=axes(fig,'Position',[0.055 0.18 0.74 0.75]); hold(ax,'on');
axis(ax,'equal'); grid(ax,'on'); xlabel(ax,'X [m]'); ylabel(ax,'Y [m]');
plot(ax,pista.conoIzq(:,1),pista.conoIzq(:,2),'o','Color',[0 .3 .8], ...
    'MarkerSize',3,'HandleVisibility','off');
plot(ax,pista.conoDer(:,1),pista.conoDer(:,2),'o','Color',[.9 .7 0], ...
    'MarkerSize',3,'HandleVisibility','off');
plot(ax,[pista.centro(:,1);pista.centro(1,1)], ...
    [pista.centro(:,2);pista.centro(1,2)],'k:','HandleVisibility','off');
plot(ax,pista.centro(1,1),pista.centro(1,2),'ks','MarkerFaceColor','g', ...
    'MarkerSize',6,'HandleVisibility','off');
xy=[pista.conoIzq;pista.conoDer];
for i=1:n, xy=[xy;resultados{i}.z(:,1:2)]; end %#ok<AGROW>
xlim(ax,[min(xy(:,1))-5 max(xy(:,1))+5]);
ylim(ax,[min(xy(:,2))-5 max(xy(:,2))+5]);
coche=gobjects(n,1); estela=coche; flecha=coche; estadoTexto=coche;
infoAx=axes(fig,'Position',[0.81 0.20 0.18 0.70], ...
    'XLim',[0 1],'YLim',[0 1],'Visible','off');
text(infoAx,0,0.98,'Seguimiento de trayectoria','FontWeight','bold','FontSize',11);
text(infoAx,0,0.91,{'Rectangulo: orientacion psi';'Flecha: velocidad real'; ...
    'Tamano visual: 2.7 x 1.2 m'},'FontSize',9,'VerticalAlignment','top');
% Geometria exclusivamente visual, centrada en el CG.
L=2.7; W=1.2;
for i=1:n
    c=colores(mod(i-1,size(colores,1))+1,:);
    estela(i)=plot(ax,NaN,NaN,'-','Color',c,'LineWidth',1.5);
    coche(i)=hgtransform('Parent',ax);
    patch('Parent',coche(i),'XData',[-L/2 L/2 L/2 -L/2], ...
        'YData',[-W/2 -W/2 W/2 W/2],'ZData',zeros(1,4), ...
        'FaceColor',c,'FaceAlpha',0.85,'EdgeColor',[.1 .1 .1],'LineWidth',1);
    patch('Parent',coche(i),'XData',[0.4 L/2 0.4], ...
        'YData',[-W/3 0 W/3],'ZData',[.01 .01 .01], ...
        'FaceColor','w','EdgeColor','none');
    flecha(i)=quiver(ax,0,0,0,0,0,'Color',c,'LineWidth',1.4,'MaxHeadSize',0.8);
    estadoTexto(i)=text(infoAx,0,0.72-(i-1)*(0.70/n),'', ...
        'Color',c,'FontSize',9,'VerticalAlignment','top','Interpreter','none');
end
reloj=annotation(fig,'textbox',[0.06 0.94 0.9 0.04], ...
    'String','','EdgeColor','none','FontSize',14,'FontWeight','bold');
annotation(fig,'textbox',[0.06 0.11 0.91 0.04], ...
    'String','Cada coche termina donde acaba su historico; no se extrapola su movimiento.', ...
    'EdgeColor','none','FontSize',10);
tiempoActual=0; velocidad=1; ultimoTic=tic; temporizador=[];
boton=[]; deslizador=[]; lista=[];

if exportando
    escritor=VideoWriter(opciones.video,'MPEG-4');
    escritor.FrameRate=opciones.fps; escritor.Quality=90;
    open(escritor); cierre=onCleanup(@() close(escritor));
    for k=1:numel(tiempos)
        actualizar(tiempos(k)); drawnow;
        % Render propio de MATLAB: no usa captura de pantalla de Windows.
        rgb=print(fig,'-RGBImage','-r90');
        rgb=rgb(1:2*floor(size(rgb,1)/2),1:2*floor(size(rgb,2)/2),:);
        writeVideo(escritor,rgb);
        if mod(k,120)==0, fprintf('Renderizado: %.0f %%\n',100*k/numel(tiempos)); end
    end
    clear cierre;
    delete(fig);
    fprintf('Video generado: %s (%d fotogramas)\n',opciones.video,numel(tiempos));
    fig=[];
    return
end

boton=uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position',[.06 .045 .10 .045],'String','Reproducir','Callback',@alternar);
uicontrol(fig,'Style','pushbutton','Units','normalized', ...
    'Position',[.17 .045 .09 .045],'String','Reiniciar','Callback',@reiniciar);
uicontrol(fig,'Style','text','Units','normalized','Position',[.28 .055 .07 .025], ...
    'String','Velocidad','BackgroundColor','w');
lista=uicontrol(fig,'Style','popupmenu','Units','normalized', ...
    'Position',[.35 .048 .08 .04],'String',{'0.25x','0.5x','1x','2x','4x'}, ...
    'Value',3,'Callback',@cambiarVelocidad);
deslizador=uicontrol(fig,'Style','slider','Units','normalized', ...
    'Position',[.46 .055 .48 .025],'Min',0,'Max',max(T,eps),'Value',0, ...
    'SliderStep',[min(1,1/max(T,eps)) min(1,5/max(T,eps))],'Callback',@buscarTiempo);
temporizador=timer('ExecutionMode','fixedSpacing','Period',1/opciones.fps, ...
    'BusyMode','drop','TimerFcn',@avanzar,'ErrorFcn',@errorAnimacion);
fig.CloseRequestFcn=@cerrar;
fig.DeleteFcn=@liberarTimer;
actualizar(0);
if opciones.reproducir
    boton.String='Pausar'; ultimoTic=tic; start(temporizador);
end

    function actualizar(t)
        if ~isgraphics(fig), return; end
        [~,k]=min(abs(tiempos-t));
        reloj.String=sprintf('Planificador V9 + Pure Pursuit     t = %.2f / %.2f s',t,T);
        for ii=1:n
            h=resultados{ii}; d=datos{ii}(k,:);
            coche(ii).Matrix=makehgtform('translate',[d(1) d(2) 0])*makehgtform('zrotate',d(3));
            estela(ii).XData=datos{ii}(1:k,1); estela(ii).YData=datos{ii}(1:k,2);
            finalizado=t>=h.t(end);
            vxGlobal=d(4)*cos(d(3))-d(5)*sin(d(3));
            vyGlobal=d(4)*sin(d(3))+d(5)*cos(d(3));
            flecha(ii).XData=d(1); flecha(ii).YData=d(2);
            if finalizado
                flecha(ii).UData=0; flecha(ii).VData=0;
                texto=sprintf('Vmax: %.0f m/s\nENSAYO FINALIZADO\n%s\nUltima rapidez: %.2f m/s', ...
                    h.v_set,h.motivo,hypot(d(4),d(5)));
            else
                flecha(ii).UData=0.35*vxGlobal; flecha(ii).VData=0.35*vyGlobal;
                texto=sprintf('Vmax: %.0f m/s\nReferencia PI: %.2f m/s\nRapidez: %.2f m/s\nRumbo: %.1f grados', ...
                    h.v_set,d(6),hypot(d(4),d(5)),rad2deg(d(3)));
            end
            estadoTexto(ii).String=texto;
        end
        if ~isempty(deslizador) && isgraphics(deslizador), deslizador.Value=min(t,T); end
    end

    function avanzar(~,~)
        paso=toc(ultimoTic); ultimoTic=tic;
        tiempoActual=min(T,tiempoActual+paso*velocidad);
        actualizar(tiempoActual); drawnow limitrate;
        if tiempoActual>=T
            stop(temporizador); boton.String='Reproducir';
        end
    end
    function alternar(~,~)
        if strcmp(temporizador.Running,'on')
            stop(temporizador); boton.String='Reproducir';
        else
            if tiempoActual>=T, tiempoActual=0; actualizar(0); end
            ultimoTic=tic; start(temporizador); boton.String='Pausar';
        end
    end
    function reiniciar(~,~)
        tiempoActual=0; ultimoTic=tic; actualizar(0);
    end
    function cambiarVelocidad(~,~)
        valores=[.25 .5 1 2 4]; velocidad=valores(lista.Value); ultimoTic=tic;
    end
    function buscarTiempo(~,~)
        tiempoActual=deslizador.Value; ultimoTic=tic; actualizar(tiempoActual);
    end
    function errorAnimacion(~,~)
        if isgraphics(boton), boton.String='Reproducir'; end
        liberarTimer();
    end
    function liberarTimer(varargin)
        if ~isempty(temporizador) && isvalid(temporizador)
            stop(temporizador); delete(temporizador);
        end
    end
    function cerrar(~,~)
        liberarTimer(); delete(fig);
    end
end
