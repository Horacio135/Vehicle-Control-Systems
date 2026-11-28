function Renderizar_lazo_completo_33_2D
% Contorno naranja punteado: misma huella del coche en la pose estimada.
base=fileparts(mfilename('fullpath'));raiz=fileparts(fileparts(base));addpath(raiz,fileparts(base),'-begin');salida=fullfile(raiz,'Resultados','Lazo_completo_33_curvas');if ~isfolder(salida),mkdir(salida);end
datos=fullfile(salida,'Lazo_completo_33_curvas.mat');if ~isfile(datos),datos=fullfile(raiz,'..','Results','Extreme_33_Corners','Complete_Loop','Lazo_completo_33_curvas.mat');end;d=load(datos);h=d.h;p=d.p;pista=d.pista;
fig=figure('Visible','off','Color','w','Position',[20 20 1440 900],'MenuBar','none','ToolBar','none');
ax=axes(fig,'Position',[.06 .17 .90 .72]);hold(ax,'on');axis(ax,'equal');grid(ax,'on');
plot(ax,pista.bordeIzq(:,1),pista.bordeIzq(:,2),'Color',[.65 .65 .65]);
plot(ax,pista.bordeDer(:,1),pista.bordeDer(:,2),'Color',[.65 .65 .65]);
plot(ax,pista.conoIzq(:,1),pista.conoIzq(:,2),'^','MarkerSize',5,'Color',[.95 .5 .03]);
plot(ax,pista.conoDer(:,1),pista.conoDer(:,2),'^','MarkerSize',5,'Color',[.1 .35 .9]);
traza=plot(ax,nan,nan,'Color',[.1 .35 .85]);
trazaEst=plot(ax,nan,nan,':','Color',[.9 .35 .02],'LineWidth',1);
forma=[1 -.7;1 .7;-1 .7;-1 -.7;1 -.7];
coche=patch(ax,nan,nan,[.05 .35 .85],'EdgeColor',[.03 .15 .3],'LineWidth',1);
frente=plot(ax,nan,nan,'w-','LineWidth',3);
ruedas=gobjects(4,1);for j=1:4,ruedas(j)=patch(ax,nan,nan,[.08 .08 .08]);end
estimado=plot(ax,nan,nan,':','Color',[1 .3 .02],'LineWidth',3);
frenteEst=plot(ax,nan,nan,':','Color',[1 .3 .02],'LineWidth',2);
legend(ax,[coche estimado traza trazaEst],{'Vehículo real','Contorno estimado','Trayectoria real','Trayectoria estimada'},'Location','northeast','AutoUpdate','off');
xlabel(ax,'X [m]');ylabel(ax,'Y [m]');
titulo=annotation(fig,'textbox',[.02 .93 .96 .055],'EdgeColor','none','FontSize',15,'FontWeight','bold','HorizontalAlignment','center');
hud=annotation(fig,'textbox',[.04 .015 .94 .11],'EdgeColor','none','BackgroundColor','w','FontSize',12);
fps=20;video=VideoWriter(fullfile(salida,'Lazo_completo_33_curvas_2D_real_y_estimado_x1.mp4'),'MPEG-4');video.FrameRate=fps;video.Quality=95;open(video);
tiempos=unique([0:1/fps:h.t(end) h.t(end)]);
try
 for i=1:numel(tiempos)
  t=tiempos(i);k=min(numel(h.t),floor(t/p.dt)+1);z=h.z(k,:);e=h.estimado(k,:);
  R=[cos(z(3)) -sin(z(3));sin(z(3)) cos(z(3))];xy=forma*R'+z(1:2);
  set(coche,'XData',xy(:,1),'YData',xy(:,2));nariz=[.85 -.45;.85 .45]*R'+z(1:2);set(frente,'XData',nariz(:,1),'YData',nariz(:,2));
  dw=atan(p.L./(p.L/tan(h.delta(k))-p.yw));if abs(h.delta(k))<1e-9,dw=zeros(1,4);end
  for j=1:4
   giro=z(3);if j<=2,giro=giro+dw(j);end
   Rw=[cos(giro) -sin(giro);sin(giro) cos(giro)];centro=[p.xw(j) p.yw(j)]*R'+z(1:2);
   pol=[-.23 -.09;.23 -.09;.23 .09;-.23 .09]*Rw'+centro;set(ruedas(j),'XData',pol(:,1),'YData',pol(:,2));
  end
  Re=[cos(e(3)) -sin(e(3));sin(e(3)) cos(e(3))];xy=forma*Re'+e(1:2);
  set(estimado,'XData',xy(:,1),'YData',xy(:,2));nariz=[.85 -.45;.85 .45]*Re'+e(1:2);set(frenteEst,'XData',nariz(:,1),'YData',nariz(:,2));
  ids=max(1,k-4000):10:k;set(traza,'XData',h.z(ids,1),'YData',h.z(ids,2));set(trazaEst,'XData',h.estimado(ids,1),'YData',h.estimado(ids,2));
  centro=(z(1:2)+e(1:2))/2;errorPos=norm(e(1:2)-z(1:2));escala=max(15,errorPos*.8+3);
  xlim(ax,centro(1)+[-1 1]*escala*1.6);ylim(ax,centro(2)+[-1 1]*escala);
  titulo.String=sprintf('Lazo completo: sensores + estimador + ABS + TC + ESC | t = %.2f s | x1',t);
  estado='Simulación en curso';if i==numel(tiempos),estado=h.motivo;end
  hud.String=sprintf('Velocidad real %.1f m/s | estimada %.1f m/s | avance %.1f %% | error de posición %.3f m\nABS %d | TC %d | ESC %d | %s\nQ = diag([0.001 0.001 0.00001 0.003 0.003].^2). Contorno naranja: pose estimada, sin acceso a la verdad de planta.', ...
   z(4),e(4),100*h.avance(k)/pista.longitud,errorPos,h.ABS(k),h.TC(k),h.ESC(k),estado);
  drawnow;frame=getframe(fig);writeVideo(video,frame);
  if i==min(241,numel(tiempos)),imwrite(frame.cdata,fullfile(salida,'Previa_lazo_completo.png'));end
  if mod(i,400)==0,fprintf('Render lazo completo: %d/%d\n',i,numel(tiempos));end
 end
 close(video);close(fig);
catch fallo
 close(video);if isgraphics(fig),close(fig);end;rethrow(fallo);
end
r=VideoReader(fullfile(salida,'Lazo_completo_33_curvas_2D_real_y_estimado_x1.mp4'));
assert(abs(r.Duration-h.t(end))<.2);for f=[0 .5 .99],r.CurrentTime=f*(r.Duration-1/r.FrameRate);assert(~isempty(readFrame(r)));end
fprintf('Vídeo verificado: %.3f s, %.0f fps; %s\n',r.Duration,r.FrameRate,h.motivo);
end
