%% Verificación de máscaras, trazas y conservación de la ley ESC V1
base=fileparts(mfilename('fullpath'));raiz=fileparts(base);addpath(base);
for sub={'Control','Dinamica','Configuracion'},addpath(fullfile(raiz,sub{1}),'-begin');end
datos=fullfile(raiz,'Resultados','Circuito_extremo_33_curvas','Comparacion_completa.mat');if ~isfile(datos),datos=fullfile(raiz,'..','Results','Extreme_33_Corners','Perfect_State','Comparacion_completa.mat');end;d=load(datos);
assert(height(d.pista.curvas)==33);
for j=1:4
 h=d.historias{j};assert(all(isfinite(h.z),'all'));
 assert(~any(any(h.fuerzas>0,2)&any(h.fuerzas<0,2)));
 assert(max(abs(h.fuerzas(:,3)-h.fuerzas(:,4)).*all(h.fuerzas>=0,2))<1e-10);
 if ~d.ayudas(j).ABS,assert(~any(h.ABS));end
 if ~d.ayudas(j).TC,assert(~any(h.TC));end
 if ~d.ayudas(j).ESC,assert(~any(h.ESC));end
 ds=diff(h.avance);assert(all(abs(ds)<1),'Salto de proyección de trayectoria');
end
original=[];extraido=[];h=d.historias{4};
for k=1:13:numel(h.t)
 z=h.z(k,:);est=struct('vx',z(4),'vy',z(5),'r',z(6),'ax',z(7),'ay',z(8),'zControl',z(1:20)');
 [~,original,a]=controlESC(est,h.delta(k),zeros(1,4),d.p,original,d.p.dt);
 [f,extraido,b]=solicitarCorreccionESC(est,h.delta(k),d.p,extraido,d.p.dt);
 assert(isequal(f,a.frenoPrevioABS));assert(isequal(a.rRef,b.rRef));
 assert(isequal(a.momentoSolicitado,b.momentoSolicitado));assert(isequal(a.rueda,b.rueda));
end
disp('Verificado: cuatro configuraciones, fuerzas, proyección y ley ESC V1 conservada.');
