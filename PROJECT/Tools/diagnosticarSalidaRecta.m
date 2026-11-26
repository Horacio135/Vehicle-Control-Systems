base=fileparts(mfilename('fullpath'));
proyecto=fileparts(fileparts(fileparts(mfilename('fullpath'))));
d=load(fullfile(proyecto,'PROJECT/Results/Closed_Loop/histories.mat'));
a=load(fullfile(proyecto,'PROJECT/Results/Ackermann/histories.mat'),'sinControl');
casos={d.sinControl,d.conControl,a.sinControl};
nombres={'Estimado sin ABS/TC','Estimado con ABS/TC','Perfecto sin ABS/TC previo'};
eventos=[];muestras=[];
for j=1:3
 h=casos{j};
 if isfield(h,'error_estimacion'),ep=vecnorm(h.error_estimacion(:,1:2),2,2);evx=abs(h.error_estimacion(:,4));epsi=abs(h.error_estimacion(:,3));
 else,ep=zeros(size(h.t));evx=ep;epsi=ep;end
 fuera=find(abs(h.error)>1.5,1);vxCero=find(h.t>1 & h.z(:,4)<1e-10,1);
 granError=find(evx>1,1);errorPsi=find(epsi>deg2rad(5),1);
 fprintf('\n%s\n',nombres{j});
 fprintf('Primera salida centro >1.5 m: ');if isempty(fuera),disp('no');else,disp(h.t(fuera));end
 fprintf('Primer vx=0 después de 1 s: ');if isempty(vxCero),disp('no');else,disp(h.t(vxCero));end
 fprintf('Primer error vx >1 m/s: ');if isempty(granError),disp('no');else,disp(h.t(granError));end
 fprintf('Primer error psi >5 grados: ');if isempty(errorPsi),disp('no');else,disp(h.t(errorPsi));end
 if ~isempty(fuera)
    k=fuera;
    fprintf('En primera salida: X %.3f Y %.3f vx %.3f vy %.3f psi %.3f delta %.3f grados, errorPos %.3f, errVx %.3f\n', ...
      h.z(k,1),h.z(k,2),h.z(k,4),h.z(k,5),rad2deg(h.z(k,3)),rad2deg(h.delta(k)),ep(k),evx(k));
 end
 if j==1
    for t=[0 .5 1 2 3 4 5 6 7 8 9 10 12]
      [~,k]=min(abs(h.t-t));
      muestras=[muestras;h.t(k),h.z(k,1),h.z(k,2),h.error(k),ep(k),rad2deg(h.error_estimacion(k,3)), ...
          h.z(k,4),h.estimado(k,4),h.z(k,5),rad2deg(h.delta(k)),mean(h.kappa(k,3:4)),sum(h.FxSolicitadas(k,:))]; %#ok<AGROW>
    end
 end
end
tabla=array2table(muestras,'VariableNames',{'t','X','Y','ErrorTray','ErrorPos','ErrorPsi_deg','vx','vxEst','vy','delta_deg','kappaTras','Fsol'});
disp(tabla);writetable(tabla,fullfile(proyecto,'PROJECT','Current State','Resultados','diagnostico_recta_sin_control.csv'));
