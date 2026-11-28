%% Comprobaciones y demanda original tras cada recuperación
base=fileparts(mfilename('fullpath'));raiz=fileparts(fileparts(base));addpath(fileparts(base),raiz,'-begin');salida=fullfile(raiz,'Resultados','Comparacion_ESC_recuperacion');if ~isfolder(salida),mkdir(salida);end
datos=fullfile(salida,'Comparacion_completa.mat');if isfile(datos),d=load(datos);else,publicados=fullfile(raiz,'..','Results','Extreme_33_Corners','ESC_Recovery');d=load(fullfile(publicados,'Contexto_comparacion.mat'));a=load(fullfile(publicados,'Caso_1.mat'),'h');b=load(fullfile(publicados,'Caso_2.mat'),'h');d.historias={a.h,b.h};end
assert(numel(d.historias)==2&&all([d.ayudas.ABS])&&all([d.ayudas.TC]));
assert(~d.ayudas(1).ESC&&d.ayudas(2).ESC);
fig=figure('Visible','off','Color','w','Position',[20 20 1500 850]);tiledlayout(2,1);
for j=1:2
 h=d.historias{j};assert(strcmp(h.motivo,'Vuelta completada'),h.motivo);assert(abs(h.error(end))<1);
 assert(all(isfinite(h.z),'all'));assert(~any(any(h.fuerzas>0,2)&any(h.fuerzas<0,2)));
 % No hay recolocación de posición: desplazamiento compatible con velocidad.
 desplazamiento=vecnorm(diff(h.z(:,1:2)),2,2);
 limite=(vecnorm(h.z(1:end-1,4:5),2,2)+2)*d.p.dt;
 assert(all(desplazamiento<limite));
 if j==1,assert(~any(h.ESC));end
 ids=1:20:numel(h.t);nexttile;plot(h.t(ids),h.z(ids,4),'LineWidth',1);hold on;
 plot(h.t(ids),h.vRef(ids),'--');grid on;legend('Velocidad real','Referencia solicitada');
 title(sprintf('%s | %.3f s | %d recuperaciones',d.nombres{j},h.t(end),h.recuperaciones));xlabel('Tiempo [s]');ylabel('Velocidad [m/s]');
 fprintf('%s: %.3f s, %d recuperaciones, avance %.3f%%\n',d.nombres{j},h.t(end),h.recuperaciones,100*h.avance(end)/d.pista.longitud);
end
exportgraphics(fig,fullfile(salida,'Velocidad_y_recuperaciones.png'),'Resolution',150);close(fig);
disp('Verificado: vuelta completa, ayudas correctas y posición continua sin reinicios.');
