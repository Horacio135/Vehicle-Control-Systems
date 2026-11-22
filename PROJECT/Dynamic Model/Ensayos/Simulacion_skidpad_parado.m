function [histSkid0Cin, histSkid0Din] = Simulacion_skidpad_parado(p, ctrl, geom, dt, fnSimularE2, fnCinematico, fnDinamico)

% Estado inicial con velocidad nula
z0Skid = [0; -geom.R; 0; 0; 0; 0];

% Duracion ampliada para incluir la aceleracion
TmaxSkid = 25;

% Modelo cinematico
histSkid0Cin = fnSimularE2(z0Skid, p, ctrl, geom, dt, TmaxSkid, fnCinematico);

% Modelo dinamico
histSkid0Din = fnSimularE2(z0Skid, p, ctrl, geom, dt, TmaxSkid, fnDinamico);

% Representacion de resultados
figure('Name','Skidpad desde parado','Color','w');

subplot(2,2,1)
plot(histSkid0Cin.X,histSkid0Cin.Y,'b--','LineWidth',1.3);
hold on;
plot(histSkid0Din.X,histSkid0Din.Y,'r','LineWidth',1.3);
axis equal; grid on;
xlabel('X [m]'); ylabel('Y [m]');
title('Trayectorias desde parado');
legend('Cinematico','Dinamico','Location','best');

subplot(2,2,2)
plot(histSkid0Cin.t,histSkid0Cin.vx,'b--', ...
    histSkid0Din.t,histSkid0Din.vx,'r','LineWidth',1.3);
yline(ctrl.v_skid,'k:');
grid on;
xlabel('Tiempo [s]'); ylabel('v_x [m/s]');
title('Aceleracion desde parado');
legend('Cinematico','Dinamico','Referencia');

subplot(2,2,3)
plot(histSkid0Cin.t,histSkid0Cin.r,'b--', ...
    histSkid0Din.t,histSkid0Din.r,'r','LineWidth',1.3);
grid on;
xlabel('Tiempo [s]'); ylabel('r [rad/s]');
title('Velocidad de guinada');
legend('Cinematico','Dinamico');

subplot(2,2,4)
plot(histSkid0Din.t,histSkid0Din.eta_slip(:,3:4),...
    'LineWidth',1.3);
grid on;
xlabel('Tiempo [s]');
ylabel('Factor lateral');
ylim([0 1.05]);
title('Adherencia lateral trasera');
legend('TI','TD');

fprintf('\n--- SKIDPAD DESDE PARADO ---\n');
fprintf('Reduccion lateral maxima: %.1f %%\n', ...
    100*(1-min(histSkid0Din.eta_slip(:,3:4),[],'all')));

end