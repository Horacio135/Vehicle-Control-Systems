function hist = Simulacion_aceleracion_frenada(p, dt, fnDinamico)
% Full-demand launch to 35 m/s followed by maximum braking, without ABS.
% Reuses pasoDinamico with 6 motion states and 14 memory components in z.
% Call from the main script after defining the parameters:
% histRecta = Simulacion_aceleracion_frenada(p, dt, @pasoDinamico);
% Force saturation does NOT establish mechanical wheel lock: this model
% does not calculate angular velocities or actual longitudinal slip.
% Wheel order: DI, DD, TI, TD. Forces use tyre coordinates.

if nargin < 2 || isempty(dt), dt = 0.002; end
validateattributes(dt, {'numeric'}, {'scalar','real','finite','positive'});
if nargin < 3 || ~isa(fnDinamico, 'function_handle')
    error('Supply @pasoDinamico from the main script.');
end

vObjetivo = 35;                     % [m/s]
vParada = 0.01;                     % stopping threshold [m/s]
Tmax = 120;                         % safety time limit [s]
N = ceil(Tmax/dt);
z = [zeros(6,1); zeros(2,1); ones(4,1); ones(4,1); zeros(4,1)];
frenando = false;
tFrenada = NaN;

hist.t = zeros(N,1);
hist.z = zeros(N,numel(z));
hist.Fx_sol = zeros(N,1);
hist.Fxw_sol = zeros(N,4);
hist.Fxw = zeros(N,4);
hist.Fyw = zeros(N,4);
hist.Fzw = zeros(N,4);
hist.lambda_w = zeros(N,4);
hist.eta_brake = ones(N,4);
hist.s_brake_w = zeros(N,4);
hist.Fmax_peak_w = zeros(N,4);
hist.bloqueo_w = false(N,4);
hist.mu_eff_w = zeros(N,4);
hist.aprov_long = zeros(N,4);
hist.aprov_total = zeros(N,4);
hist.ax = zeros(N,1);
hist.frenando = false(N,1);

for k = 1:N
    t = (k-1)*dt;
    if ~frenando && z(4) >= vObjetivo
        frenando = true;
        tFrenada = t;
    end
    if frenando
        FxSol = -p.Fx_brake_max;
        reparto = p.brake_w;
    else
        FxSol = p.Fx_drive_max;
        reparto = p.drive_w;
    end

    % State and forces correspond to the START of the same time step.
    [zNext, tele] = fnDinamico(z, [FxSol; 0], p, dt);
    hist.t(k) = t;
    hist.z(k,:) = z.';
    hist.Fx_sol(k) = FxSol;
    hist.Fxw_sol(k,:) = FxSol .* reparto;
    hist.Fxw(k,:) = tele.Fxw;
    hist.Fyw(k,:) = tele.Fyw;
    hist.Fzw(k,:) = tele.Fzw;
    hist.lambda_w(k,:) = tele.lambda_w;
    hist.eta_brake(k,:) = tele.eta_brake;
    hist.s_brake_w(k,:) = tele.s_brake_w;
    hist.Fmax_peak_w(k,:) = tele.Fmax_peak_w;
    hist.bloqueo_w(k,:) = tele.bloqueo_w;
    hist.mu_eff_w(k,:) = tele.mu_eff_w;
    hist.aprov_long(k,:) = abs(tele.Fxw) ./ max(tele.Fmax_peak_w, eps);
    hist.aprov_total(k,:) = hypot(tele.Fxw,tele.Fyw) ./ max(tele.Fmax_peak_w, eps);
    hist.ax(k) = tele.ax;
    hist.frenando(k) = frenando;

    % Stop before integrating beyond the stopping threshold, avoiding
    % artificially maintained braking demand at vx = 0.
    if frenando && z(4) <= vParada
        break
    end
    z = zNext;
    end

campos = fieldnames(hist);
for j = 1:numel(campos)
    hist.(campos{j}) = hist.(campos{j})(1:k,:);
end
hist.tInicioFrenada = tFrenada;
hist.vObjetivo = vObjetivo;
hist.paradaAlcanzada = frenando && hist.z(end,4) <= vParada;
if ~frenando
    warning('35 m/s was not reached within %.1f s.', Tmax);
elseif ~hist.paradaAlcanzada
    warning('The stopping criterion was not reached within %.1f s.', Tmax);
end

% Include the switching instant in both curves to avoid visual gaps.
acel = ~hist.frenando;
freno = hist.frenando;
idxFreno = find(freno,1);
if ~isempty(idxFreno), acel(idxFreno) = true; end
colores = lines(4);
nombres = {'DI','DD','TI','TD'};
figure('Name','Straight: acceleration to 35 m/s and maximum braking','Color','w');

subplot(3,1,1); hold on;
plot(hist.t(acel),hist.z(acel,4),'b-','LineWidth',1.5,'DisplayName','Acceleration');
plot(hist.t(freno),hist.z(freno,4),'b--','LineWidth',1.5,'DisplayName','Braking');
yline(vObjetivo,'k:','35 m/s','HandleVisibility','off');
grid on; ylabel('v_x [m/s]'); title('Longitudinal speed');
legend('Location','best');

subplot(3,1,2); hold on;
for i = 1:4
    plot(hist.t(acel),hist.aprov_long(acel,i),'-','Color',colores(i,:), ...
        'LineWidth',1.3,'DisplayName',nombres{i});
    plot(hist.t(freno),hist.aprov_long(freno,i),'--','Color',colores(i,:), ...
        'LineWidth',1.3,'HandleVisibility','off');
end
yline(1,'k:','Limit','HandleVisibility','off');
grid on; ylabel('|F_{x,i}| / (\mu_i F_{z,i})');
title('Longitudinal utilization: solid = acceleration; dashed = braking');
legend('Location','best'); ylim([0 1.1]);

subplot(3,1,3); hold on;
for i = 1:4
    plot(hist.t(acel),hist.Fxw(acel,i),'-','Color',colores(i,:), ...
        'LineWidth',1.3,'DisplayName',nombres{i});
    plot(hist.t(freno),hist.Fxw(freno,i),'--','Color',colores(i,:), ...
        'LineWidth',1.3,'HandleVisibility','off');
end
yline(0,'k:','HandleVisibility','off');
grid on; xlabel('Time [s]'); ylabel('F_{x,i} [N]');
title('Effective wheel forces: positive driving, negative braking');
legend('Location','best');
for j = 1:3
    subplot(3,1,j);
    if isfinite(tFrenada)
        xline(tFrenada,'k:','Braking starts','HandleVisibility','off');
    end
end

% In this straight-line test, X advance equals travelled distance.
hist.distancia = hist.z(:,1)-hist.z(1,1);
figure('Name','Travelled distance - acceleration and braking','Color','w');
hold on;
plot(hist.t(acel),hist.distancia(acel),'b-', ...
    'LineWidth',1.5,'DisplayName','Acceleration');
plot(hist.t(freno),hist.distancia(freno),'b--', ...
    'LineWidth',1.5,'DisplayName','Braking');
if isfinite(tFrenada)
    xline(tFrenada,'k:','Braking starts','HandleVisibility','off');
end
grid on; xlabel('Time [s]'); ylabel('Distance from launch [m]');
title('Total travelled distance'); legend('Location','best');

figure('Name','Adhesion capacity and braking mode','Color','w');
subplot(2,1,1);
plot(hist.t,hist.eta_brake,'LineWidth',1.2); grid on;
ylabel('mu_{ef} / mu_{base}'); ylim([0 1.05]);
title('Available capacity; not the same as transmitted force');
legend('DI','DD','TI','TD','Location','best');
subplot(2,1,2);
stairs(hist.t,double(hist.bloqueo_w),'LineWidth',1.1); grid on;
xlabel('Time [s]'); ylabel('Approximate lock mode'); ylim([-0.05 1.05]);
legend('DI','DD','TI','TD','Location','best');

fprintf('\n--- STRAIGHT-LINE ACCELERATION AND BRAKING ---\n');
fprintf('Maximum recorded speed: %.3f m/s\n', max(hist.z(:,4)));
if frenando
    fprintf('Acceleration time: %.3f s\n', tFrenada);
    fprintf('Recorded braking time: %.3f s\n', hist.t(end)-tFrenada);
    fprintf('Braking distance: %.3f m\n', hist.z(end,1)-hist.z(idxFreno,1));
end
fprintf('Final speed: %.4f m/s\n', hist.z(end,4));
fprintf('Saturation does not establish lock: wheel rotation is not modelled.\n');
fprintf('Left/right curves coincide by straight-line symmetry.\n');
end
