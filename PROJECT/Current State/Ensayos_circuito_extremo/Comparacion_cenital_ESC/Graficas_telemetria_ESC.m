function Graficas_telemetria_ESC(origen,salida)
% Gráficas complementarias completas, con fuerza nominal y efectiva por rueda.
d=load(origen,'h','p');h=d.h;p=d.p;if ~isfolder(salida),mkdir(salida);end
nominal=max(h.nominal,0).*p.drive_w+min(h.nominal,0).*p.brake_w;
fig=figure('Visible','off','Color','w','Position',[20 20 1500 950]);tiledlayout(2,2);
ruedas={'Delantera izquierda','Delantera derecha','Trasera izquierda','Trasera derecha'};ids=1:5:numel(h.t);
for j=1:4
 nexttile;plot(h.t(ids),nominal(ids,j),'Color',[.6 .6 .6]);hold on;
 plot(h.t(ids),h.fuerzas(ids,j),'Color',[.1 .3 .9]);plot(h.t(ids),h.Fx(ids,j),'Color',[.1 .65 .25]);
 plot(h.t(ids),h.capacidad(ids,j),'k--');plot(h.t(ids),-h.capacidad(ids,j),'k--');
 grid on;title(ruedas{j});xlabel('Tiempo [s]');ylabel('Fuerza longitudinal [N]');
 legend('Nominal del conductor','Tras ABS/TC/ESC','Transmitida Fx','+Fmax efectiva','-Fmax efectiva','Location','best','FontSize',8);
end
exportgraphics(fig,fullfile(salida,'Fuerzas_por_rueda.png'),'Resolution',150);close(fig);
fig=figure('Visible','off','Color','w','Position',[20 20 1500 900]);tiledlayout(4,1);
for j=1:3
 nexttile;campos={'ABS','TC','ESC'};stairs(h.t,double(h.(campos{j})));ylim([-.1 1.1]);yticks([0 1]);grid on;ylabel(campos{j});
end
nexttile;plot(h.t,sum(h.Fx,2));hold on;plot(h.t,h.nominal);grid on;xlabel('Tiempo [s]');ylabel('Fuerza [N]');legend('Transmitida','Nominal');
exportgraphics(fig,fullfile(salida,'Activacion_y_fuerza_vuelta_completa.png'),'Resolution',150);close(fig);
end
