proyecto=fileparts(fileparts(fileparts(mfilename('fullpath'))));
addpath(fullfile(proyecto,'PROJECT','Current State','Estimacion'),'-begin');
d=load(fullfile(proyecto,'PROJECT/Results/Closed_Loop/histories.mat'));
h=d.conControl;N=numel(h.t);valores=zeros(2,5);
for j=1:2
 p=d.p;if j==1,p.Q=diag([.02 .02 .005 .10 .10].^2);end
 mem=struct('xh',h.z(1,1:5).','P',eye(5)*1e-3);salida=zeros(N,5);
 for k=1:N
  m=h.medidas(k,:);
  med=struct('acc',m(1),'acc_x',m(1),'acc_y',m(2),'gyro',m(3), ...
    'gps_X',m(4),'gps_Y',m(5),'delta',m(6),'hay_gps',logical(m(7)),'hay_direccion',isfinite(m(6)));
  paso=p.dt;if k==1,paso=0;end
  [e,mem]=estimadorEstado(med,[],p,mem,paso);
  salida(k,:)=[e.X e.Y e.psi e.vx e.vy];
 end
 error=salida-h.z(:,1:5);error(:,3)=atan2(sin(error(:,3)),cos(error(:,3)));
 valores(j,:)=[sqrt(mean(sum(error(:,1:2).^2,2))),rad2deg(sqrt(mean(error(:,3).^2))), ...
   sqrt(mean(error(:,4).^2)),sqrt(mean(error(:,5).^2)),max(vecnorm(error(:,1:2),2,2))];
 if j==2,assert(max(abs(salida-h.estimado(:,1:5)),[],'all')<1e-10);end
end
tabla=array2table(valores,'VariableNames',{'RMSE_pos_m','RMSE_psi_deg','RMSE_vx_ms','RMSE_vy_ms','Max_pos_m'}, ...
 'RowNames',{'Q anterior mismas medidas','Q actual mismas medidas'});
disp(tabla);
h=d.sinControl;k=find(abs(h.error)>1.5,1);
utilizacion=abs(h.Fxw(k,3:4))./h.capacidad(k,3:4);
margen=sqrt(max(h.capacidad(k,3:4).^2-h.Fxw(k,3:4).^2,0));
disp(struct('t_primera_salida',h.t(k),'utilizacion_x_trasera',utilizacion,'margen_lateral_trasero_N',margen));
writetable(tabla,fullfile(proyecto,'PROJECT','Current State','Resultados','comparacion_Q_mismas_medidas.csv'),'WriteRowNames',true);
