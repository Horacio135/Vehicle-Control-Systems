%% Mapa con márgenes y numeración de curvas
base=fileparts(mfilename('fullpath'));raiz=fileparts(base);salida=fullfile(raiz,'Resultados','Circuito_extremo_33_curvas');if ~isfolder(salida),mkdir(salida);end
datos=fullfile(salida,'Comparacion_completa.mat');if ~isfile(datos),datos=fullfile(raiz,'..','Results','Extreme_33_Corners','Perfect_State','Comparacion_completa.mat');end;d=load(datos,'pista','perfil');pista=d.pista;
fig=figure('Visible','off','Color','w','Position',[20 20 1500 1000]);
plot(pista.bordeIzq(:,1),pista.bordeIzq(:,2),'Color',[.6 .6 .6]);hold on;
plot(pista.bordeDer(:,1),pista.bordeDer(:,2),'Color',[.6 .6 .6]);
scatter(pista.centro(:,1),pista.centro(:,2),7,d.perfil.v,'filled');axis equal;grid on;colorbar;
xlim([min(pista.centro(:,1))-50 max(pista.centro(:,1))+50]);
ylim([min(pista.centro(:,2))-50 max(pista.centro(:,2))+50]);
title(sprintf('Circuito extremo: 33 curvas, %.1f m | referencia en m/s',pista.longitud));xlabel('X [m]');ylabel('Y [m]');
exportgraphics(fig,fullfile(salida,'Mapa_circuito.png'),'Resolution',150);close(fig);
