function Renderizar_alce_ESC_V1_13m_3D_naranja_giro15_opuesto
% Renderiza exclusivamente el MAT guardado. No vuelve a simular la dinámica.
% Cámara: traslación XY con el coche ESC; orientación, altura y zoom fijos.
carpeta=fileparts(mfilename('fullpath'));
origen=fullfile(carpeta,'Ensayos_estabilidad.mat');
hashAntes=hashArchivo(origen);d=load(origen,'historias');hs=d.historias;
assert(numel(hs)==2 && hs{1}.config.longitud==13 && hs{2}.config.ESC);
desplazamientoCamara=[-12*.88 -7*.88 20*(1-2/5)]; % Detrás respecto a +X; no respecto al rumbo.
% Giro opuesto:15 grados antihorarios vistos desde arriba alrededor de +Z.
anguloGiro=15;rotacionCamara=[cosd(anguloGiro) -sind(anguloGiro);sind(anguloGiro) cosd(anguloGiro)];
desplazamientoCamara(1:2)=(rotacionCamara*desplazamientoCamara(1:2).').';
alturaObjetivo=.30;anguloVision=48;fps=25;
fig=figure('Visible','off','Color',[.94 .96 .98],'Position',[20 20 1280 720], ...
    'MenuBar','none','ToolBar','none','Renderer','opengl');
limpiar=onCleanup(@()close(fig));
ax=axes(fig,'Position',[0 .13 1 .80]);hold(ax,'on');
patch(ax,[-100 250 250 -100],[-100 -100 100 100],[-.04 -.04 -.04 -.04], ...
    [.80 .85 .79],'EdgeColor','none');
ref=hs{1}.ref;ref=ref(ref(:,1)<=145,:);n=size(ref,1);
izq=ref+[0 1.5];der=ref-[0 1.5];
patch(ax,'Vertices',[[izq;der] zeros(2*n,1)], ...
    'Faces',[(1:n-1)' (2:n)' (n+2:2*n)' (n+1:2*n-1)'], ...
    'FaceColor',[.39 .42 .45],'EdgeColor','none');
plot3(ax,izq(:,1),izq(:,2),.004+zeros(n,1),'w-','LineWidth',1);
plot3(ax,der(:,1),der(:,2),.004+zeros(n,1),'w-','LineWidth',1);
plot3(ax,ref(:,1),ref(:,2),.008+zeros(n,1),'--','Color',[.83 .85 .86]);
ids=1:40:n;dibujarConos(ax,izq(ids,:),[1 .58 .04]);dibujarConos(ax,der(ids,:),[.08 .43 .94]);
xo=40+hs{1}.config.longitud+5;caja(ax,[xo-2 -1 .01],[4 2 .65],[.40 .42 .44]);
text(ax,xo,0,.70,'OBSTÁCULO','HorizontalAlignment','center','Color','w','FontSize',9);
car=hgtransform('Parent',ax);
caja(car,[-1 -.48 .22],[2 .96 .27],[.03 .38 .84]);
caja(car,[-.42 -.36 .49],[.95 .72 .23],[.14 .23 .33]);
caja(car,[.78 -.40 .50],[.13 .80 .04],[.95 .96 1]);
caja(car,[-.93 -.58 .42],[.16 1.16 .07],[.02 .15 .30]);
ruedas=gobjects(1,4);centros=[.70 .51;.70 -.51;-.70 .51;-.70 -.51];
for j=1:4
    ruedas(j)=hgtransform('Parent',car);
    caja(ruedas(j),[-.24 -.09 .07],[.48 .18 .30],[.05 .05 .06]);
end
carSin=hgtransform('Parent',ax);
naranja=[1 .43 .06];
cuerpoSin=caja(carSin,[-1 -.48 .22],[2 .96 .27],naranja);
caja(carSin,[-.42 -.36 .49],[.95 .72 .23],naranja);
caja(carSin,[.78 -.40 .50],[.13 .80 .04],naranja);
caja(carSin,[-.93 -.58 .42],[.16 1.16 .07],naranja);
ruedasSin=gobjects(1,4);
for j=1:4
    ruedasSin(j)=hgtransform('Parent',carSin);
    caja(ruedasSin(j),[-.24 -.09 .07],[.48 .18 .30],naranja);
end
set(findall(carSin,'Type','patch'),'FaceAlpha',.32,'EdgeColor',[.85 .28 .02], ...
    'EdgeAlpha',.40,'LineWidth',.5);
rumbo=plot3(ax,nan,nan,nan,'Color',[.62 .01 .03],'LineWidth',2);
velocidad=quiver3(ax,0,0,.07,0,0,0,0,'Color',[1 .17 .08], ...
    'LineWidth',2,'MaxHeadSize',.65);
trazaSin=plot3(ax,nan,nan,nan,'--','Color',[.84 .06 .10],'LineWidth',1.8);
trazaCon=plot3(ax,nan,nan,nan,'Color',[.03 .34 .92],'LineWidth',1.8);
legend(ax,[trazaCon cuerpoSin velocidad],{'Coche con ESC','Coche sin ESC (transparente)','Velocidad sin ESC'}, ...
    'Location','northeast','FontSize',10,'AutoUpdate','off');
axis(ax,'equal');axis(ax,'off');
set(ax,'Projection','perspective','CameraUpVector',[0 0 1], ...
    'CameraViewAngle',anguloVision,'CameraViewAngleMode','manual', ...
    'CameraPositionMode','manual','CameraTargetMode','manual','CameraUpVectorMode','manual');
light(ax,'Position',[-50 -50 150],'Style','infinite');lighting(ax,'gouraud');
encabezado=annotation(fig,'textbox',[.02 .94 .96 .055],'EdgeColor','none', ...
    'HorizontalAlignment','center','FontSize',14,'FontWeight','bold');
pie=annotation(fig,'textbox',[.025 .005 .95 .12],'EdgeColor','none', ...
    'HorizontalAlignment','left','FontSize',11,'BackgroundColor',[.94 .96 .98]);
nombre=fullfile(carpeta,'Alce_ESC_V1_13m_3D_naranja_giro15_opuesto.mp4');
v=VideoWriter(nombre,'MPEG-4');v.FrameRate=fps;v.Quality=95;open(v);
cerrar=onCleanup(@()close(v));
tiempos=unique([0:1/fps:hs{2}.t(end),hs{2}.t(end)]);
direcciones=zeros(numel(tiempos),3);alturas=zeros(numel(tiempos),1);
for i=1:numel(tiempos)
    t=tiempos(i);estados=zeros(2,3);indices=zeros(1,2);
    for j=1:2
        h=hs{j};consulta=min(t,h.t(end));
        estados(j,:)=interp1(h.t,[h.z(:,1:2) unwrap(h.z(:,3))],consulta,'linear');
        indices(j)=find(h.t<=consulta,1,'last');
    end
    pos=estados(2,1:2);psi=estados(2,3);
    car.Matrix=makehgtform('translate',[pos 0],'zrotate',psi);
    delta=hs{2}.delta(indices(2));
    for j=1:4
        giro=0;
        if j<=2 && abs(delta)>1e-8
            giro=atan(hs{2}.p.L/(hs{2}.p.L/tan(delta)-hs{2}.p.yw(j)));
        end
        ruedas(j).Matrix=makehgtform('translate',[centros(j,:) 0],'zrotate',giro);
    end
    psiSin=estados(1,3);rot=[cos(psiSin) sin(psiSin);-sin(psiSin) cos(psiSin)];
    carSin.Matrix=makehgtform('translate',[estados(1,1:2) 0],'zrotate',psiSin);
    deltaSin=hs{1}.delta(indices(1));
    for j=1:4
        giro=0;
        if j<=2 && abs(deltaSin)>1e-8
            giro=atan(hs{1}.p.L/(hs{1}.p.L/tan(deltaSin)-hs{1}.p.yw(j)));
        end
        ruedasSin(j).Matrix=makehgtform('translate',[centros(j,:) 0],'zrotate',giro);
    end
    nariz=[0 0;1.7 0]*rot+estados(1,1:2);
    set(rumbo,'XData',nariz(:,1),'YData',nariz(:,2),'ZData',[.055 .055]);
    h=hs{1};k=indices(1);c=cos(psiSin);s=sin(psiSin);
    vel=[c*h.z(k,4)-s*h.z(k,5),s*h.z(k,4)+c*h.z(k,5)];
    flecha=3*vel/max(norm(vel),1e-9);
    set(velocidad,'XData',estados(1,1),'YData',estados(1,2),'ZData',.07, ...
        'UData',flecha(1),'VData',flecha(2),'WData',0);
    for j=1:2
        h=hs{j};k=indices(j);muestras=unique([1:5:k k]);
        if j==1,objeto=trazaSin;else,objeto=trazaCon;end
        set(objeto,'XData',h.z(muestras,1),'YData',h.z(muestras,2), ...
            'ZData',.018*ones(numel(muestras),1));
    end
    % Traslación pura: posición y objetivo se desplazan exactamente lo mismo.
    objetivo=[pos alturaObjetivo];camara=[pos 0]+desplazamientoCamara;
    xlim(ax,pos(1)+[-45 45]);ylim(ax,pos(2)+[-35 35]);zlim(ax,[-.1 30]);
    campos={'CameraPosition','CameraTarget','CameraUpVector','CameraViewAngle'};
    set(ax,campos,{camara,objetivo,[0 0 1],anguloVision});
    direcciones(i,:)=ax.CameraTarget-ax.CameraPosition;alturas(i)=ax.CameraPosition(3);
    etiqueta='Sin ESC en movimiento';
    if t>hs{1}.t(end),etiqueta=sprintf('Sin ESC abortado a %.3f s: coche naranja congelado',hs{1}.t(end));end
    encabezado.String=sprintf('ESC V1 original | Transiciones13m | t=%.2fs | x1',t);
    pie.String=sprintf('Con ESC: v=%.1f m/s, beta=%.1f°  |  Sin ESC: v=%.1f m/s, beta=%.1f°\n%s\nCámara de orientación fija; sigue XY del coche azul. La flecha roja permite distinguir rumbo y derrape.', ...
        hs{2}.rapidez(indices(2)),rad2deg(hs{2}.beta(indices(2))), ...
        hs{1}.rapidez(indices(1)),rad2deg(hs{1}.beta(indices(1))),etiqueta);
    drawnow;fotograma=getframe(fig);writeVideo(v,fotograma);
    if abs(t-3.36)<.015,imwrite(fotograma.cdata,fullfile(carpeta,'Alce_ESC_V1_13m_3D_naranja_giro15_opuesto_previa.png'));end
    if mod(i,50)==0,fprintf('3D: %d/%d fotogramas\n',i,numel(tiempos));end
end
clear cerrar;
assert(max(abs(direcciones-direcciones(1,:)),[],'all')<1e-10);
assert(max(abs(alturas-alturas(1)))<1e-10);
assert(strcmp(hashAntes,hashArchivo(origen)),'El MAT original ha cambiado.');
lector=VideoReader(nombre);
for fraccion=[0 .5 .99]
    lector.CurrentTime=fraccion*(lector.Duration-1/lector.FrameRate);assert(~isempty(readFrame(lector)));
end
informe=struct('simulacion_repetida',false,'coche_sin_ESC_3D',true,'opacidad_coche_sin_ESC',.32, ...
    'reduccion_altura_camara_pct',40,'reduccion_distancia_horizontal_pct',12,'hash_datos_origen',hashAntes, ...
    'datos_sin_modificar',true,'camara_rotacion_constante',true,'camara_altura_constante',true, ...
    'camara_desplazamiento',desplazamientoCamara,'giro_vertical_antihorario_grados',15,'objetivo_altura_m',alturaObjetivo, ...
    'angulo_vision_grados',anguloVision,'fps',lector.FrameRate,'duracion_video_s',lector.Duration, ...
    'ancho',lector.Width,'alto',lector.Height,'tiempo_aborto_sin_ESC_s',hs{1}.t(end), ...
    'coche_sin_ESC_congelado_tras_aborto',true,'fisica','Dinámica plana original; 3D solo de visualización');
fid=fopen(fullfile(carpeta,'Verificacion_render_3D_naranja_giro15_opuesto.json'),'w','n','UTF-8');
fprintf(fid,'%s',jsonencode(informe,PrettyPrint=true));fclose(fid);disp(informe);disp(nombre);
end

function objeto=caja(padre,origen,tamano,color)
vertices=[0 0 0;1 0 0;1 1 0;0 1 0;0 0 1;1 0 1;1 1 1;0 1 1].*tamano+origen;
objeto=patch('Parent',padre,'Vertices',vertices,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8], ...
    'FaceColor',color,'EdgeColor','none','FaceLighting','gouraud');
end

function dibujarConos(ax,posiciones,color)
n=12;theta=(0:n-1)'*2*pi/n;vertices=[];caras=[];
for k=1:size(posiciones,1)
    desplazamiento=size(vertices,1);base=[posiciones(k,:)+.15*[cos(theta) sin(theta)],.01*ones(n,1)];
    vertices=[vertices;base;posiciones(k,:) .46]; %#ok<AGROW>
    caras=[caras;desplazamiento+[(1:n)' [2:n 1]' (n+1)*ones(n,1)]]; %#ok<AGROW>
end
patch(ax,'Vertices',vertices,'Faces',caras,'FaceColor',color,'EdgeColor','none');
end

function valor=hashArchivo(nombre)
fid=fopen(nombre,'r');limpiar=onCleanup(@()fclose(fid));b=fread(fid,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');md.update(b);
valor=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
end
