base=fileparts(fileparts(fileparts(mfilename('fullpath'))));salida=fileparts(mfilename('fullpath'));
d=load(fullfile(salida,'Ensayos_estabilidad.mat'));historias=d.historias;
assert(d.casos(1).velocidad==20 && d.casos(1).longitud==13);
for j=1:2
 h=historias{j};assert(all(h.fuerzaPrevia>=0));
 negativas=any(h.fuerzasSolicitadas<0,2);assert(all(h.ESC(negativas)));
 if j==1,assert(~any(negativas));end
end
assert(isequaln(historias{1}.ref,historias{2}.ref));verificacion=struct([]);
for j=1:2
 h=historias{j};ref=h.ref;ids=1:40:size(ref,1);conos=[ref(ids,:)+[0 1.5];ref(ids,:)-[0 1.5]];
 choques=false(size(conos,1),1);errorCentro=zeros(size(h.t));margen=zeros(size(h.t));choqueObst=false(size(h.t));
 u=linspace(-1,1,21)';v=linspace(-.6,.6,13)';perimetro=[u .6+0*u;u -.6+0*u;1+0*v v;-1+0*v v];
 for k=1:numel(h.t)
  pos=h.z(k,1:2);psi=h.z(k,3);c=cos(psi);s=sin(psi);rot=[c s;-s c];
  puntos=perimetro*rot+pos;
  centroRef=interp1(ref(:,1),ref(:,2),pos(1),'linear','extrap');errorCentro(k)=pos(2)-centroRef;
  yref=interp1(ref(:,1),ref(:,2),puntos(:,1),'linear','extrap');margen(k)=min(1.5-abs(puntos(:,2)-yref));
  locales=(conos-pos)*rot.';dx=max(abs(locales(:,1))-1,0);dy=max(abs(locales(:,2))-.6,0);
  choques=choques|(hypot(dx,dy)<=.15);
  dx=(40+h.config.longitud+5)-pos(1);dy=-pos(2);
  choqueObst(k)=abs(dx)<=2+abs(c)+.6*abs(s) && abs(dy)<=1+abs(s)+.6*abs(c) && abs(dx*c+dy*s)<=1+2*abs(c)+abs(s) && abs(-dx*s+dy*c)<=.6+2*abs(s)+abs(c);
 end
 h.errorCentro=errorCentro;h.margenCarroceria=margen;historias{j}=h;
 r=struct('ESC',logical(j==2),'finalizado',h.z(end,1)>119,'tiempo_s',h.t(end),'beta_max_grados',rad2deg(max(abs(h.beta))),'errorCentroMax_m',max(abs(errorCentro)),'margenCarroceriaMin_m',min(margen),'conos_contactados',sum(choques),'contactoObstaculo',any(choqueObst),'dentroCorredor',all(margen>=0),'dimensionesCarroceria_m',[2 1.2],'radioCono_m',.15);
 if j==1,verificacion=r;else,verificacion(j)=r;end;disp(r);
end
% La verificación publicada no modifica el MAT de simulación.
fid=fopen(fullfile(salida,'Verificacion_corredor.json'),'w','n','UTF-8');fprintf(fid,'%s',jsonencode(verificacion,PrettyPrint=true));fclose(fid);


assert(~verificacion(1).finalizado && verificacion(1).beta_max_grados>60);
assert(verificacion(2).finalizado);
