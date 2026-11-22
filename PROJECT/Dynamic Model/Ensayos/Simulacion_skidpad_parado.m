function [histSkid0Cin, histSkid0Din] = Simulacion_skidpad_parado(p, ctrl, geom, dt, fnSimularE2, fnCinematico, fnDinamico)

% Initial state at zero speed
z0Skid = [0; -geom.R; 0; 0; 0; 0];

% Longer duration to include acceleration
TmaxSkid = 25;

% Kinematic model
histSkid0Cin = fnSimularE2(z0Skid, p, ctrl, geom, dt, TmaxSkid, fnCinematico);

% Dynamic model
histSkid0Din = fnSimularE2(z0Skid, p, ctrl, geom, dt, TmaxSkid, fnDinamico);

% Results display
figure('Name','Skidpad from rest','Color','w');

subplot(2,2,1)
plot(histSkid0Cin.X,histSkid0Cin.Y,'b--','LineWidth',1.3);
hold on;
plot(histSkid0Din.X,histSkid0Din.Y,'r','LineWidth',1.3);
axis equal; grid on;
xlabel('X [m]'); ylabel('Y [m]');
title('Trajectories from rest');
legend('Kinematic','Dynamic','Location','best');

subplot(2,2,2)
plot(histSkid0Cin.t,histSkid0Cin.vx,'b--', ...
    histSkid0Din.t,histSkid0Din.vx,'r','LineWidth',1.3);
yline(ctrl.v_skid,'k:');
grid on;
xlabel('Time [s]'); ylabel('v_x [m/s]');
title('Acceleration from rest');
legend('Kinematic','Dynamic','Reference');

subplot(2,2,3)
plot(histSkid0Cin.t,histSkid0Cin.r,'b--', ...
    histSkid0Din.t,histSkid0Din.r,'r','LineWidth',1.3);
grid on;
xlabel('Time [s]'); ylabel('r [rad/s]');
title('Yaw rate');
legend('Kinematic','Dynamic');

subplot(2,2,4)
plot(histSkid0Din.t,histSkid0Din.eta_slip(:,3:4),...
    'LineWidth',1.3);
grid on;
xlabel('Time [s]');
ylabel('Lateral factor');
ylim([0 1.05]);
title('Rear lateral adhesion');
legend('TI','TD');

fprintf('\n--- SKIDPAD FROM REST ---\n');
fprintf('Maximum lateral reduction: %.1f %%\n', ...
    100*(1-min(histSkid0Din.eta_slip(:,3:4),[],'all')));

end