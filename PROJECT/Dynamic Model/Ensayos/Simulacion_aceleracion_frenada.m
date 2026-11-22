function hist = Simulacion_aceleracion_frenada(p, dt, fnDinamico)
% Arranque a plena demanda hasta 35 m/s y frenada maxima, sin ABS.
% Reutiliza pasoDinamico con 6 estados de movimiento y 14 memorias en z.
% Uso desde el principal (con los parametros ya definidos):
% histRecta = Simulacion_aceleracion_frenada(p, dt, @pasoDinamico);
% La saturacion de fuerza NO demuestra bloqueo de rueda: este modelo no
% calcula velocidades angulares ni deslizamiento longitudinal real.
% Orden de ruedas: DI, DD, TI, TD. Fuerzas en ejes del neumatico.

if nargin < 2 || isempty(dt), dt = 0.002; end
validateattributes(dt, {'numeric'}, {'scalar','real','finite','positive'});
if nargin < 3 || ~isa(fnDinamico, 'function_handle')
    error('Proporciona @pasoDinamico desde el archivo principal.');
end

vObjetivo = 35;                     % [m/s]
vParada = 0.01;                     % Umbral de parada [m/s]
Tmax = 120;                         % Limite de seguridad [s]
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

    % Estado y fuerzas correspondientes al INICIO del mismo paso.
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

    % Se termina antes de integrar mas alla del umbral de parada, evitando
    % que la demanda de frenado se mantenga artificialmente con vx = 0.
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
    warning('No se alcanzaron 35 m/s en %.1f s.', Tmax);
elseif ~hist.paradaAlcanzada
    warning('No se alcanzo la parada en %.1f s.', Tmax);
end

% Incluir el instante de conmutacion en ambas curvas evita huecos visuales.
acel = ~hist.frenando;
freno = hist.frenando;
idxFreno = find(freno,1);
if ~isempty(idxFreno), acel(idxFreno) = true; end
colores = lines(4);
nombres = {'DI','DD','TI','TD'};
figure('Name','Recta: aceleracion a 35 m/s y frenada maxima','Color','w');

subplot(3,1,1); hold on;
plot(hist.t(acel),hist.z(acel,4),'b-','LineWidth',1.5,'DisplayName','Aceleracion');
plot(hist.t(freno),hist.z(freno,4),'b--','LineWidth',1.5,'DisplayName','Frenada');
yline(vObjetivo,'k:','35 m/s','HandleVisibility','off');
grid on; ylabel('v_x [m/s]'); title('Velocidad longitudinal');
legend('Location','best');

subplot(3,1,2); hold on;
for i = 1:4
    plot(hist.t(acel),hist.aprov_long(acel,i),'-','Color',colores(i,:), ...
        'LineWidth',1.3,'DisplayName',nombres{i});
    plot(hist.t(freno),hist.aprov_long(freno,i),'--','Color',colores(i,:), ...
        'LineWidth',1.3,'HandleVisibility','off');
end
yline(1,'k:','Limite','HandleVisibility','off');
grid on; ylabel('|F_{x,i}| / (\mu_i F_{z,i})');
title('Aprovechamiento longitudinal: continua = aceleracion; discontinua = frenada');
legend('Location','best'); ylim([0 1.1]);

subplot(3,1,3); hold on;
for i = 1:4
    plot(hist.t(acel),hist.Fxw(acel,i),'-','Color',colores(i,:), ...
        'LineWidth',1.3,'DisplayName',nombres{i});
    plot(hist.t(freno),hist.Fxw(freno,i),'--','Color',colores(i,:), ...
        'LineWidth',1.3,'HandleVisibility','off');
end
yline(0,'k:','HandleVisibility','off');
grid on; xlabel('Tiempo [s]'); ylabel('F_{x,i} [N]');
title('Fuerzas efectivas por rueda: positivas en traccion, negativas en frenada');
legend('Location','best');
for j = 1:3
    subplot(3,1,j);
    if isfinite(tFrenada)
        xline(tFrenada,'k:','Inicio frenada','HandleVisibility','off');
    end
end

% En este ensayo rectilineo, el avance X es la distancia recorrida.
hist.distancia = hist.z(:,1)-hist.z(1,1);
figure('Name','Distancia recorrida - aceleracion y frenada','Color','w');
hold on;
plot(hist.t(acel),hist.distancia(acel),'b-', ...
    'LineWidth',1.5,'DisplayName','Aceleracion');
plot(hist.t(freno),hist.distancia(freno),'b--', ...
    'LineWidth',1.5,'DisplayName','Frenada');
if isfinite(tFrenada)
    xline(tFrenada,'k:','Inicio frenada','HandleVisibility','off');
end
grid on; xlabel('Tiempo [s]'); ylabel('Distancia desde el arranque [m]');
title('Distancia total recorrida'); legend('Location','best');

figure('Name','Capacidad de adherencia y modo de frenado','Color','w');
subplot(2,1,1);
plot(hist.t,hist.eta_brake,'LineWidth',1.2); grid on;
ylabel('mu_{ef} / mu_{base}'); ylim([0 1.05]);
title('Capacidad disponible; no equivale a fuerza ejercida');
legend('DI','DD','TI','TD','Location','best');
subplot(2,1,2);
stairs(hist.t,double(hist.bloqueo_w),'LineWidth',1.1); grid on;
xlabel('Tiempo [s]'); ylabel('Bloqueo aproximado'); ylim([-0.05 1.05]);
legend('DI','DD','TI','TD','Location','best');

fprintf('\n--- ACELERACION Y FRENADA EN RECTA ---\n');
fprintf('Velocidad maxima registrada: %.3f m/s\n', max(hist.z(:,4)));
if frenando
    fprintf('Tiempo acelerando: %.3f s\n', tFrenada);
    fprintf('Tiempo de frenada registrado: %.3f s\n', hist.t(end)-tFrenada);
    fprintf('Distancia de frenada: %.3f m\n', hist.z(end,1)-hist.z(idxFreno,1));
end
fprintf('Velocidad final: %.4f m/s\n', hist.z(end,4));
fprintf('Saturacion no equivale a bloqueo: no se modela rotacion de ruedas.\n');
fprintf('Las curvas izquierda/derecha coinciden por simetria en recta.\n');
end
