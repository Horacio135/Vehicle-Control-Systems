function Renderizar_telemetria_ESC(tipo,origen,salida,prueba)
% Visualización sincronizada de datos guardados, sin repetir ni alterar física.
base=fileparts(mfilename('fullpath'));raiz=fileparts(fileparts(base));if nargin<4,prueba=false;end
if nargin<2,origen=fullfile(raiz,'..','Results','Extreme_33_Corners','ESC_Recovery','Caso_2.mat');end
if nargin<3,salida=fullfile(raiz,'Resultados','Telemetria_ESC');end
if ~isfolder(salida),mkdir(salida);end
d=load(origen,'h','p');h=d.h;p=d.p;
assert(h.ayudas.ABS&&h.ayudas.TC&&h.ayudas.ESC&&strcmp(h.motivo,'Vuelta completada'));
archivoGeometria=fullfile(fileparts(origen),'Comparacion_completa.mat');if ~isfile(archivoGeometria),archivoGeometria=fullfile(fileparts(origen),'Contexto_comparacion.mat');end;geometria=load(archivoGeometria,'pista');pista=geometria.pista;
fps=20;ventana=8;es3D=strcmpi(tipo,'3D');colores=lines(4);
fig=figure('Visible','off','Color','w','Position',[20 20 1440 900],'MenuBar','none','ToolBar','none','GraphicsSmoothing','off');
ax=axes(fig,'Position',[.025 .19 .53 .70],'Clipping','on','ClippingStyle','rectangle');hold(ax,'on');axis(ax,'equal');grid(ax,'on');
if es3D
 n=size(pista.centro,1);vertices=[[pista.bordeIzq;pista.bordeDer] zeros(2*n,1)];
 caras=[(1:n)' mod((1:n)',n)+1 mod((1:n)',n)+1+n (1:n)'+n];
 patch(ax,'Vertices',vertices,'Faces',caras,'FaceColor',[.4 .43 .46],'EdgeColor','none');
 plot3(ax,pista.bordeIzq(:,1),pista.bordeIzq(:,2),.01*ones(n,1),'w');
 plot3(ax,pista.bordeDer(:,1),pista.bordeDer(:,2),.01*ones(n,1),'w');
 conos(ax,pista.conoIzq,[1 .55 .03]);conos(ax,pista.conoDer,[.05 .35 .9]);
 set(ax,'Projection','perspective');axis(ax,'off');
 light(ax,'Position',[-20 -30 80],'Style','infinite');lighting(ax,'gouraud');
else
 plot(ax,pista.bordeIzq(:,1),pista.bordeIzq(:,2),'Color',[.6 .6 .6]);
 plot(ax,pista.bordeDer(:,1),pista.bordeDer(:,2),'Color',[.6 .6 .6]);
 plot(ax,pista.conoIzq(:,1),pista.conoIzq(:,2),'^','MarkerSize',4,'Color',[1 .55 .03]);
 plot(ax,pista.conoDer(:,1),pista.conoDer(:,2),'^','MarkerSize',4,'Color',[.05 .35 .9]);
 xlabel(ax,'X [m]');ylabel(ax,'Y [m]');
end
coche=hgtransform('Parent',ax);caja(coche,[-1 -.48 .20],[2 .96 .27],[.03 .38 .84]);
caja(coche,[-.42 -.34 .47],[.9 .68 .23],[.14 .23 .33]);
caja(coche,[.82 -.4 .48],[.12 .8 .04],[.9 .95 1]);
ruedas=gobjects(1,4);
for j=1:4
 ruedas(j)=hgtransform('Parent',coche);caja(ruedas(j),[-.23 -.1 .06],[.46 .2 .3],[.05 .05 .06]);
end
traza=plot3(ax,nan,nan,nan,'Color',[.02 .3 .9],'LineWidth',1.2);
if ~es3D,view(ax,2);end
tit=annotation(fig,'textbox',[.02 .94 .96 .055],'EdgeColor','none','FontSize',15,'FontWeight','bold','HorizontalAlignment','center');
hud=annotation(fig,'textbox',[.02 .015 .53 .105],'EdgeColor','none','FontSize',11,'BackgroundColor','w');
panel=gobjects(6,1);lineas=cell(6,1);cursores=gobjects(6,1);datos=cell(6,1);
datos{1}=double(h.ABS);datos{2}=double(h.TC);datos{3}=double(h.ESC);
datos{4}=[h.nominal sum(h.fuerzas,2) sum(h.Fx,2)];
datos{5}=h.capacidad;datos{6}=rad2deg(h.delta);
posiciones=[.61 .80 .37 .085;.61 .685 .37 .085;.61 .57 .37 .085;.61 .355 .37 .155;.61 .145 .37 .155;.61 .035 .37 .065];
titulos={'ABS: intervención efectiva','TC: intervención efectiva','ESC: frenada selectiva efectiva', ...
 'Fuerza longitudinal total (ejes de rueda)','Capacidad combinada efectiva por neumático: Fmax','Pure Pursuit: ángulo solicitado'};
for j=1:6
 panel(j)=axes(fig,'Position',posiciones(j,:));hold(panel(j),'on');grid(panel(j),'on');
 lineas{j}=plot(panel(j),nan(2,size(datos{j},2)),nan(2,size(datos{j},2)),'LineWidth',1.15);
 title(panel(j),titulos{j},'FontSize',10);cursores(j)=xline(panel(j),0,':k');
 if j<=3,ylim(panel(j),[-.1 1.1]);yticks(panel(j),[0 1]);ylabel(panel(j),'0 / 1');
 elseif j==4
  ylabel(panel(j),'N');legend(panel(j),lineas{j},{'Nominal','Corregida','Transmitida'},'FontSize',9,'Location','northwest','Orientation','horizontal','AutoUpdate','off');
  valores=datos{j};lim=[min(valores,[],'all') max(valores,[],'all')];ylim(panel(j),lim+[-1 1]*.15*diff(lim));
 elseif j==5
  ylabel(panel(j),'N');legend(panel(j),lineas{j},{'DI','DD','TI','TD'},'FontSize',8,'Location','northwest','Orientation','horizontal');
  ylim(panel(j),[0 max(datos{j},[],'all')*1.12]);for w=1:4,lineas{j}(w).Color=colores(w,:);end
 else,ylabel(panel(j),'grados');ylim(panel(j),[-30 30]);xlabel(panel(j),'Tiempo de simulación [s]');end
end
nombre=fullfile(salida,['ESC_ABS_TC_' upper(tipo) '_telemetria_x1.mp4']);
% Rasterizar las curvas una vez por ventana; evitar redibujar seis ejes
% en cada fotograma. La máscara revela exclusivamente el pasado.
firma=hashArchivo(origen);archivoCache=fullfile(salida,['Paneles_' firma(1:16) '.mat']);
if isfile(archivoCache)
 cache=load(archivoCache,'fondos','curvas','W','H','columnaInicio');
 fondos=cache.fondos;curvas=cache.curvas;W=cache.W;H=cache.H;columnaInicio=cache.columnaInicio;
else
 fondos=cell(1,ceil(h.t(end)/ventana));curvas=fondos;
 for b=1:numel(fondos)
  inicio=(b-1)*ventana;ids=find(h.t>=inicio&h.t<=inicio+ventana);
  for j=1:6
   cursores(j).Visible='off';xlim(panel(j),[inicio inicio+ventana]);
   for w=1:numel(lineas{j}),set(lineas{j}(w),'XData',h.t(ids),'YData',datos{j}(ids,w));end
  end
  foto=getframe(fig);[H,W,~]=size(foto.cdata);columnaInicio=floor(.56*W)+1;
  curvas{b}=foto.cdata(:,columnaInicio:end,:);
  for j=1:6,for w=1:numel(lineas{j}),set(lineas{j}(w),'XData',nan,'YData',nan);end;end
  foto=getframe(fig);fondos{b}=foto.cdata(:,columnaInicio:end,:);
  fprintf('Paneles: %d/%d ventanas\n',b,numel(fondos));
 end
 save(archivoCache,'fondos','curvas','W','H','columnaInicio','-v7.3');
end
delete(panel);
if prueba,nombre=strrep(nombre,'.mp4','_prueba.mp4');end;video=VideoWriter(nombre,'MPEG-4');video.FrameRate=fps;video.Quality=90;open(video);
tiempos=unique([0:1/fps:h.t(end) h.t(end)]);if prueba,tiempos=12;end
try
 for i=1:numel(tiempos)
  t=tiempos(i);k=min(numel(h.t),floor(t/p.dt)+1);z=h.z(k,:);pos=z(1:2);
  coche.Matrix=makehgtform('translate',[pos 0],'zrotate',z(3));
  dw=atan(p.L./(p.L/tan(h.delta(k))-p.yw));if abs(h.delta(k))<1e-9,dw=zeros(1,4);end
  for w=1:4
   giro=0;if w<=2,giro=dw(w);end
   ruedas(w).Matrix=makehgtform('translate',[p.xw(w) p.yw(w) 0],'zrotate',giro);
  end
  ids=max(1,k-5000):15:k;set(traza,'XData',h.z(ids,1),'YData',h.z(ids,2),'ZData',.025*ones(numel(ids),1));
  if es3D
   xlim(ax,pos(1)+[-40 40]);ylim(ax,pos(2)+[-35 35]);zlim(ax,[-.1 25]);
   set(ax,'CameraPosition',[pos+[-8.6059 -8.6832] 12],'CameraTarget',[pos .3], ...
    'CameraUpVector',[0 0 1],'CameraViewAngle',48);
  else,xlim(ax,pos(1)+[-18 18]);ylim(ax,pos(2)+[-18 18]);end
  titulo=sprintf('ABS + TC + ESC | vista %s | t = %.2f s | velocidad real',upper(tipo),t);tit.String=titulo;
  hud.String=sprintf('v = %.1f m/s | referencia = %.1f m/s | avance %.1f %%\nABS %d | TC %d | ESC %d | deriva %.2f grados\nPure Pursuit manda dirección; el longitudinal solicita fuerza.\nFmax es capacidad combinada, no reserva longitudinal. Estado perfecto, sin sensores/EKF.', ...
   z(4),h.vRef(k),100*h.avance(k)/pista.longitud,h.ABS(k),h.TC(k),h.ESC(k),rad2deg(h.beta(k)));
  b=min(numel(fondos),floor(t/ventana)+1);inicio=(b-1)*ventana;derecha=fondos{b};
  for j=1:6
   x0=floor(posiciones(j,1)*W)+1-columnaInicio+1;
   x1=ceil((posiciones(j,1)+posiciones(j,3))*W)-columnaInicio+1;
   y0=max(1,round((1-posiciones(j,2)-posiciones(j,4))*H)+1);
   y1=min(H,round((1-posiciones(j,2))*H));
   corte=min(x1,max(x0,round(x0+(x1-x0)*(t-inicio)/ventana)));
   derecha(y0:y1,x0:corte,:)=curvas{b}(y0:y1,x0:corte,:);
   derecha(y0:3:y1,corte,:)=60;
  end
  drawnow;frame=getframe(fig);assert(size(frame.cdata,1)==H&&size(frame.cdata,2)==W);
  filas=round(.075*H):H;frame.cdata(filas,columnaInicio:end,:)=derecha(filas,:,:);
  writeVideo(video,frame);
  if abs(t-12)<1/(2*fps),imwrite(frame.cdata,fullfile(salida,['Previa_' upper(tipo) '.png']));end
  if mod(i,200)==0,fprintf('%s: %d/%d fotogramas\n',tipo,i,numel(tiempos));end
 end
 close(video);close(fig);
catch fallo
 close(video);if isgraphics(fig),close(fig);end;rethrow(fallo);
end
lector=VideoReader(nombre);if ~prueba,assert(lector.Duration>190&&lector.Duration<191);end
for fraccion=[0 .5 .99],lector.CurrentTime=fraccion*(lector.Duration-1/lector.FrameRate);assert(~isempty(readFrame(lector)));end
fprintf('%s verificado: %.3f s, %.0f fps\n',tipo,lector.Duration,lector.FrameRate);
end

function objeto=caja(padre,origen,tamano,color)
v=[0 0 0;1 0 0;1 1 0;0 1 0;0 0 1;1 0 1;1 1 1;0 1 1].*tamano+origen;
objeto=patch('Parent',padre,'Vertices',v,'Faces',[1 2 3 4;5 6 7 8;1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8],'FaceColor',color,'EdgeColor','none');
end
function conos(ax,pos,color)
v=zeros(5*size(pos,1),3);f=zeros(4*size(pos,1),3);
for k=1:size(pos,1)
 b=(k-1)*5;v(b+(1:5),:)=[pos(k,:)+[-.14 -.14],0;pos(k,:)+[.14 -.14],0;pos(k,:)+[.14 .14],0;pos(k,:)+[-.14 .14],0;pos(k,:),.4];
 f((k-1)*4+(1:4),:)=b+[1 2 5;2 3 5;3 4 5;4 1 5];
end
patch(ax,'Vertices',v,'Faces',f,'FaceColor',color,'EdgeColor','none');
end

function valor=hashArchivo(nombre)
f=fopen(nombre,'r');assert(f>=0);limpiar=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');md.update(b);
valor=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));
end
