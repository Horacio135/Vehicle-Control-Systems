function Simulacion_skidpad(histCin, histDin, p, geom, Rcin, Rdin, informarVueltas)
fprintf('Ackermann R = %.2f m  (delta = %.2f deg)\n', geom.R, atan((p.ld+p.lt)/geom.R)*180/pi);
fprintf('Measured radius — kinematic: %.2f m | dynamic: %.2f m\n', Rcin, Rdin);
fprintf('Understeer: dynamic radius is %.1f%% larger.\n', 100*(Rdin-Rcin)/Rcin);
fprintf('Timing (FS rule: the second lap of each circle is timed):\n');
informarVueltas('kinematic', histCin);
informarVueltas('dynamic', histDin);
figure('Name','L2 · Skidpad','Color','w');

subplot(2,2,1); hold on; axis equal; grid on;  
  th = linspace(0,2*pi,200);
  plot(geom.R_in*cos(th), geom.R_in*sin(th), 'o','Color',[0 .3 .8],'MarkerSize',3);
  plot(geom.R_out*cos(th),geom.R_out*sin(th),'o','Color',[.9 .8 0],'MarkerSize',3);
  plot(histCin.X, histCin.Y, 'b-','LineWidth',1.2);
  plot(histDin.X, histDin.Y, 'r-','LineWidth',1.2);
  legend('inner cones (blue)','outer cones (yellow)','kinematic','dynamic','Location','bestoutside');
  title('Trajectory'); xlabel('X [m]'); ylabel('Y [m]');

subplot(2,2,2);   
  plot(histCin.t,histCin.r,'b', histDin.t,histDin.r,'r','LineWidth',1.2);
  ylabel('r [rad/s]'); xlabel('t [s]'); grid on; legend('cin','din');
  title('Yaw rate');

subplot(2,2,3);   
  plot(histCin.t, histCin.vx.*histCin.r/9.81,'b', ...
       histDin.t, histDin.vx.*histDin.r/9.81,'r','LineWidth',1.2);
  ylabel('a_y [g]'); xlabel('t [s]'); grid on; legend('cin','din');
  title('Lateral acceleration');

subplot(2,2,4);   
  plot(histDin.t, histDin.af*180/pi,'r', histDin.t, histDin.ar*180/pi,'m','LineWidth',1.2);
  ylabel('\alpha [deg]'); xlabel('t [s]'); grid on; legend('\alpha_f','\alpha_r');
  title('Slip angles (dynamic; approximately zero in kinematic)');

%% NEW FIGURE: four-wheel dynamic-model telemetry
% Wheel order: DI, DD, TI, TD. Original simulation is unchanged.
figure('Name','Four-wheel model - telemetry','Color','w');
nombres = {'DI','DD','TI','TD'};

subplot(2,2,1);
plot(histDin.t, rad2deg(histDin.alpha_w), 'LineWidth',1.1);
grid on; xlabel('t [s]'); ylabel('\alpha_i [deg]');
title('Individual slip angles'); legend(nombres,'Location','best');

subplot(2,2,2);
plot(histDin.t, histDin.Fxw, 'LineWidth',1.1);
grid on; xlabel('t [s]'); ylabel('F_{x,i} [N]');
title('Longitudinal forces per wheel'); legend(nombres,'Location','best');

subplot(2,2,3);
plot(histDin.t, histDin.Fyw, 'LineWidth',1.1);
grid on; xlabel('t [s]'); ylabel('F_{y,i} [N]');
title('Lateral forces per wheel'); legend(nombres,'Location','best');

subplot(2,2,4);
plot(histDin.t, histDin.Mz_long, 'b', ...
     histDin.t, histDin.Mz_lat, 'r', ...
     histDin.t, histDin.Mz_long + histDin.Mz_lat, 'k--', 'LineWidth',1.1);
grid on; xlabel('t [s]'); ylabel('M_z [N m]');
title('Yaw-moment contributions');
legend('Longitudinal forces','Lateral forces','Total','Location','best');
%% Friction-circle check

t = histDin.t(:);
Fx = histDin.Fxw;
Fy = histDin.Fyw;
Fz = histDin.Fzw;

% Adhesion utilization per wheel
Fmax = Fz .* p.muw;
eta = hypot(Fx, Fy) ./ Fmax;

% Lateral acceleration from forces
delta = atan((p.ld + p.lt)/geom.R);
delta_w = [delta, delta, 0, 0];

Fy_chasis = Fx.*sin(delta_w) + Fy.*cos(delta_w);
ay_real = sum(Fy_chasis, 2)/(p.m*p.g);

% Plots
figure('Color','w');

subplot(2,1,1)
plot(t, eta, 'LineWidth',1.3)
yline(1,'k--','Adhesion limit');
grid on;
ylabel('Utilization \eta');
legend('DI','DD','TI','TD','Location','best');
title('Friction-circle utilization');

subplot(2,1,2)
plot(t, ay_real, 'r','LineWidth',1.5)
hold on;
plot(t, histDin.vx(:).*histDin.r(:)/p.g, 'b--');
grid on;
xlabel('Time [s]');
ylabel('a_y [g]');
legend('Actual forces','Approximation vx*r');
title('Lateral-acceleration comparison');

% Numerical check
fprintf('Maximum utilization per wheel:\n');
disp(max(eta,[],1));

fprintf('Maximum lateral acceleration: %.3f g\n', ...
    max(abs(ay_real)));
fprintf('\n--- FOUR-WHEEL MODEL SUMMARY ---\n');
fprintf('Final speed: %.2f m/s\n', histDin.vx(end));
fprintf('Final yaw rate: %.3f rad/s\n', histDin.r(end));
fprintf('Max |slip angle| per wheel [deg]:');
fprintf('%.2f ', max(abs(rad2deg(histDin.alpha_w)),[],1));
fprintf('\nMax |Fx| per wheel [N]:');
fprintf('%.1f ', max(abs(histDin.Fxw),[],1));
fprintf('\nMax |Fy| per wheel [N]:');
fprintf('%.1f ', max(abs(histDin.Fyw),[],1));
fprintf('\n');
%% Results: load transfer and cornering stiffness
t = histDin.t(:);
Fz = histDin.Fzw;
nombres = {'DI','DD','TI','TD'};

% Stiffness used at each step
Ca = p.Caw .* (Fz ./ p.Fzw0).^p.nCa;

% Effective forces and adhesion capacity
Fres = hypot(histDin.Fxw, histDin.Fyw);
Fmax = Fz .* p.muw;
eta = Fres ./ max(Fmax,eps);

figure
plot(t, Fz, 'LineWidth',1.2);
grid on;
xlabel('Time [s]'); ylabel('F_z [N]');
title('Vertical wheel load');
legend(nombres,'Location','best');
end
