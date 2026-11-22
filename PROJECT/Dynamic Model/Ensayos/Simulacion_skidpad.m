function Simulacion_skidpad(histCin, histDin, p, geom, Rcin, Rdin, informarVueltas)
fprintf('Ackermann R = %.2f m  (delta = %.2f deg)\n', geom.R, atan((p.ld+p.lt)/geom.R)*180/pi);
fprintf('Radio medido — cinemático: %.2f m | dinámico: %.2f m\n', Rcin, Rdin);
fprintf('Subviraje: el dinámico traza %.1f%% más radio.\n', 100*(Rdin-Rcin)/Rcin);
fprintf('Tiempos (regla FS: se cronometra la 2ª vuelta de cada círculo):\n');
informarVueltas('cinemático', histCin);
informarVueltas('dinámico  ', histDin);
figure('Name','L2 · Skidpad','Color','w');

subplot(2,2,1); hold on; axis equal; grid on;  
  th = linspace(0,2*pi,200);
  plot(geom.R_in*cos(th), geom.R_in*sin(th), 'o','Color',[0 .3 .8],'MarkerSize',3);
  plot(geom.R_out*cos(th),geom.R_out*sin(th),'o','Color',[.9 .8 0],'MarkerSize',3);
  plot(histCin.X, histCin.Y, 'b-','LineWidth',1.2);
  plot(histDin.X, histDin.Y, 'r-','LineWidth',1.2);
  legend('conos int (azul)','conos ext (amarillo)','cinemático','dinámico','Location','bestoutside');
  title('Trayectoria'); xlabel('X [m]'); ylabel('Y [m]');

subplot(2,2,2);   
  plot(histCin.t,histCin.r,'b', histDin.t,histDin.r,'r','LineWidth',1.2);
  ylabel('r [rad/s]'); xlabel('t [s]'); grid on; legend('cin','din');
  title('Velocidad de guiñada');

subplot(2,2,3);   
  plot(histCin.t, histCin.vx.*histCin.r/9.81,'b', ...
       histDin.t, histDin.vx.*histDin.r/9.81,'r','LineWidth',1.2);
  ylabel('a_y [g]'); xlabel('t [s]'); grid on; legend('cin','din');
  title('Aceleración lateral');

subplot(2,2,4);   
  plot(histDin.t, histDin.af*180/pi,'r', histDin.t, histDin.ar*180/pi,'m','LineWidth',1.2);
  ylabel('\alpha [deg]'); xlabel('t [s]'); grid on; legend('\alpha_f','\alpha_r');
  title('Ángulos de deriva (dinámico; en cinemático ~0)');

%% NUEVA FIGURA: telemetria del modelo dinamico de cuatro ruedas
% Orden de ruedas: DI, DD, TI, TD. No modifica la simulacion original.
figure('Name','Modelo 4 ruedas - Telemetria','Color','w');
nombres = {'DI','DD','TI','TD'};

subplot(2,2,1);
plot(histDin.t, rad2deg(histDin.alpha_w), 'LineWidth',1.1);
grid on; xlabel('t [s]'); ylabel('\alpha_i [deg]');
title('Angulos de deriva individuales'); legend(nombres,'Location','best');

subplot(2,2,2);
plot(histDin.t, histDin.Fxw, 'LineWidth',1.1);
grid on; xlabel('t [s]'); ylabel('F_{x,i} [N]');
title('Fuerzas longitudinales por rueda'); legend(nombres,'Location','best');

subplot(2,2,3);
plot(histDin.t, histDin.Fyw, 'LineWidth',1.1);
grid on; xlabel('t [s]'); ylabel('F_{y,i} [N]');
title('Fuerzas laterales por rueda'); legend(nombres,'Location','best');

subplot(2,2,4);
plot(histDin.t, histDin.Mz_long, 'b', ...
     histDin.t, histDin.Mz_lat, 'r', ...
     histDin.t, histDin.Mz_long + histDin.Mz_lat, 'k--', 'LineWidth',1.1);
grid on; xlabel('t [s]'); ylabel('M_z [N m]');
title('Contribuciones al momento de guinada');
legend('Fuerzas longitudinales','Fuerzas laterales','Total','Location','best');
%% Comprobacion del circulo de adherencia

t = histDin.t(:);
Fx = histDin.Fxw;
Fy = histDin.Fyw;
Fz = histDin.Fzw;

% Aprovechamiento de adherencia por rueda
Fmax = Fz .* p.muw;
eta = hypot(Fx, Fy) ./ Fmax;

% Aceleracion lateral a partir de las fuerzas
delta = atan((p.ld + p.lt)/geom.R);
delta_w = [delta, delta, 0, 0];

Fy_chasis = Fx.*sin(delta_w) + Fy.*cos(delta_w);
ay_real = sum(Fy_chasis, 2)/(p.m*p.g);

% Graficas
figure('Color','w');

subplot(2,1,1)
plot(t, eta, 'LineWidth',1.3)
yline(1,'k--','Limite adherencia');
grid on;
ylabel('Aprovechamiento \eta');
legend('DI','DD','TI','TD','Location','best');
title('Uso del circulo de adherencia');

subplot(2,1,2)
plot(t, ay_real, 'r','LineWidth',1.5)
hold on;
plot(t, histDin.vx(:).*histDin.r(:)/p.g, 'b--');
grid on;
xlabel('Tiempo [s]');
ylabel('a_y [g]');
legend('Fuerzas reales','Aproximacion vx*r');
title('Comparacion de aceleracion lateral');

% Comprobacion numerica
fprintf('Aprovechamiento maximo por rueda:\n');
disp(max(eta,[],1));

fprintf('Aceleracion lateral maxima: %.3f g\n', ...
    max(abs(ay_real)));
fprintf('\n--- Resumen modelo 4 ruedas ---\n');
fprintf('Velocidad final: %.2f m/s\n', histDin.vx(end));
fprintf('Guiñada final: %.3f rad/s\n', histDin.r(end));
fprintf('Max |deriva| por rueda [deg]: ');
fprintf('%.2f ', max(abs(rad2deg(histDin.alpha_w)),[],1));
fprintf('\nMax |Fx| por rueda [N]: ');
fprintf('%.1f ', max(abs(histDin.Fxw),[],1));
fprintf('\nMax |Fy| por rueda [N]: ');
fprintf('%.1f ', max(abs(histDin.Fyw),[],1));
fprintf('\n');
%% Resultados: transferencia de carga y rigidez de deriva
t = histDin.t(:);
Fz = histDin.Fzw;
nombres = {'DI','DD','TI','TD'};

% Rigidez utilizada en cada paso
Ca = p.Caw .* (Fz ./ p.Fzw0).^p.nCa;

% Fuerzas efectivas y capacidad de adherencia
Fres = hypot(histDin.Fxw, histDin.Fyw);
Fmax = Fz .* p.muw;
eta = Fres ./ max(Fmax,eps);

figure
plot(t, Fz, 'LineWidth',1.2);
grid on;
xlabel('Tiempo [s]'); ylabel('F_z [N]');
title('Carga vertical por rueda');
legend(nombres,'Location','best');
end
