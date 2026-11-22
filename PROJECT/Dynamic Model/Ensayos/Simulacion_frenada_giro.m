function [histLeve, histAcusado] = Simulacion_frenada_giro(p, dt, fnDinamico)
% Accelerate to 35 m/s, brake fully and steer after 4 m of braking.
% Steering is the road-wheel angle, not the steering-wheel angle.
% Uses pasoDinamico(z,u,p,dt) with the complete 20-component state.
% Does not automatically release the brake or alter physical parameters.
% Call from the main script:
% [histGiroLeve,histGiroAcusado] = Simulacion_frenada_giro(p,dt,@pasoDinamico);

validateattributes(dt,{'numeric'},{'scalar','real','finite','positive'});
assert(isa(fnDinamico,'function_handle'),'@pasoDinamico is required.');
config.vObjetivo = 35;             % [m/s]
config.distanciaGiro = 4;           % from braking onset [m]
config.tEntrada = 0.15;             % smooth steering application [s]
config.Tmax = 120;                 % time limit [s]
config.vParada = 0.01;             % stopping threshold [m/s]

histLeve = ejecutarCaso(p,dt,fnDinamico,config,deg2rad(2),'Mild turn (2 degrees)');
histAcusado = ejecutarCaso(p,dt,fnDinamico,config,deg2rad(10),'Sharp turn (10 degrees)');
mostrarCaso(histLeve,p);
mostrarCaso(histAcusado,p);

figure('Name','Braking with steering - comparison','Color','w');
subplot(2,1,1); hold on;
plot(histLeve.z(:,1),histLeve.z(:,2),'b-','LineWidth',1.4);
plot(histAcusado.z(:,1),histAcusado.z(:,2),'r--','LineWidth',1.4);
axis equal; grid on; xlabel('X [m]'); ylabel('Y [m]');
title('Trajectories: locking can prevent both turns');
legend('Mild','Sharp','Location','best');
subplot(2,1,2); hold on;
plot(histLeve.t,rad2deg(histLeve.delta),'b-','LineWidth',1.4);
plot(histAcusado.t,rad2deg(histAcusado.delta),'r--','LineWidth',1.4);
grid on; xlabel('Time [s]'); ylabel('Wheel steering [degrees]');
title('Steering requested after 4 m of braking');
legend('Mild','Sharp','Location','best');
end

function h = ejecutarCaso(p,dt,modelo,cfg,deltaObjetivo,nombre)
N = ceil(cfg.Tmax/dt);
z = [zeros(8,1);ones(8,1);zeros(4,1)];
distancia = 0;
frenando = false;
tFrenada = NaN; sFrenada = NaN; tGiro = NaN; sGiro = NaN;
h.t = zeros(N,1); h.z = zeros(N,20); h.distancia = zeros(N,1);
h.delta = zeros(N,1); h.Fx_sol = zeros(N,1); h.frenando = false(N,1);
campos = {'Fxw','Fyw','Fzw','Fmax_w','Fmax_peak_w','eta_brake', ...
    'eta_slip','lambda_w','factor_x_bloqueo','factor_y_bloqueo'};
for j=1:numel(campos), h.(campos{j})=zeros(N,4); end
h.bloqueo_w = false(N,4);
for k=1:N
    t=(k-1)*dt;
    if ~frenando && z(4)>=cfg.vObjetivo
        frenando=true; tFrenada=t; sFrenada=distancia;
    end
    if frenando && isnan(tGiro) && distancia-sFrenada>=cfg.distanciaGiro
        tGiro=t; sGiro=distancia;
    end
    delta=0;
    if isfinite(tGiro)
        entrada=min(max((t-tGiro)/cfg.tEntrada,0),1);
        delta=deltaObjetivo*entrada^2*(3-2*entrada);
    end
    if frenando, Fx=-p.Fx_brake_max; else, Fx=p.Fx_drive_max; end

    [zNext,tele]=modelo(z,[Fx;delta],p,dt);
    h.t(k)=t; h.z(k,:)=z.'; h.distancia(k)=distancia;
    h.delta(k)=delta; h.Fx_sol(k)=Fx; h.frenando(k)=frenando;
    for j=1:numel(campos), h.(campos{j})(k,:)=tele.(campos{j}); end
    h.bloqueo_w(k,:)=tele.bloqueo_w;
    if frenando && z(4)<=cfg.vParada, break; end
    % Accumulated path length, not distance from the origin.
    distancia=distancia+hypot(zNext(1)-z(1),zNext(2)-z(2));
    z=zNext;
end
camposHist=fieldnames(h);
for j=1:numel(camposHist), h.(camposHist{j})=h.(camposHist{j})(1:k,:); end
h.nombre=nombre; h.tInicioFrenada=tFrenada; h.sInicioFrenada=sFrenada;
h.tInicioGiro=tGiro; h.sInicioGiro=sGiro; h.deltaObjetivo=deltaObjetivo;
h.paradaAlcanzada=frenando && h.z(end,4)<=cfg.vParada;
nominal=max(h.Fmax_peak_w,eps);
h.aprov_long=abs(h.Fxw)./nominal;
h.aprov_lat=abs(h.Fyw)./nominal;
% Residual component capacity while retaining the other current force.
% Do not add these capacities: they share the same friction circle.
h.capacidad_x=sqrt(max(h.Fmax_w.^2-h.Fyw.^2,0)).*h.factor_x_bloqueo;
h.capacidad_y=sqrt(max(h.Fmax_w.^2-h.Fxw.^2,0)).* ...
    h.factor_y_bloqueo.*h.eta_slip;
h.capacidad_x_rel=h.capacidad_x./nominal;
h.capacidad_y_rel=h.capacidad_y./nominal;
if ~h.paradaAlcanzada, warning('%s: test ended before reaching the stopping criterion.',nombre); end
if isnan(tGiro), warning('%s: 4 m of braking was not reached.',nombre); end
fprintf('\n--- %s ---\n',nombre);
fprintf('Acceleration: %.3f s; braking: %.3f s; braking distance: %.3f m\n', ...
    tFrenada,h.t(end)-tFrenada,h.distancia(end)-sFrenada);
fprintf('Steering starts: %.3f m after braking begins\n',sGiro-sFrenada);
fprintf('Maximum absolute lateral force per wheel [N]:');
fprintf('%.3f ',max(abs(h.Fyw),[],1)); fprintf('\n');
end

function mostrarCaso(h,p)
colores=lines(4); nombres={'DI','DD','TI','TD'};
acel=~h.frenando; freno=h.frenando;
idx=find(freno,1); if ~isempty(idx), acel(idx)=true; end
figure('Name',['Braking with steering -' h.nombre],'Color','w', ...
    'Position',[80 80 1400 850]);
tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

nexttile; hold on;
plot(h.t(acel),h.z(acel,4),'b-','LineWidth',1.4,'DisplayName','Acceleration');
plot(h.t(freno),h.z(freno,4),'b--','LineWidth',1.4,'DisplayName','Braking');
marcasTiempo(h); grid on; xlabel('Time [s]'); ylabel('v_x [m/s]');
title('Speed / time'); legend('Location','best');

nexttile; hold on;
plot(h.distancia(acel),h.z(acel,4),'b-','LineWidth',1.4);
plot(h.distancia(freno),h.z(freno,4),'b--','LineWidth',1.4);
if isfinite(h.sInicioFrenada), xline(h.sInicioFrenada,'k:','Braking'); end
if isfinite(h.sInicioGiro), xline(h.sInicioGiro,'m:','Turn'); end
grid on; xlabel('Accumulated distance [m]'); ylabel('v_x [m/s]');
title('Speed / distance');

nexttile; hold on;
plot(h.t(acel),h.distancia(acel),'b-','LineWidth',1.4);
plot(h.t(freno),h.distancia(freno),'b--','LineWidth',1.4);
marcasTiempo(h); grid on; xlabel('Time [s]'); ylabel('Accumulated distance [m]');
title('Distance / time');

nexttile; hold on;
porRueda(h.t,h.aprov_long,acel,freno,colores,nombres);
yline(1,'k:','Nominal limit','HandleVisibility','off');
marcasTiempo(h); grid on; xlabel('Time [s]'); ylabel('|F_x| / (mu_{base} F_z)');
title('Longitudinal utilization'); legend('Location','best');

nexttile; hold on;
porRueda(h.t,h.aprov_lat,acel,freno,colores,nombres);
yline(1,'k:','Nominal limit','HandleVisibility','off');
marcasTiempo(h); grid on; xlabel('Time [s]'); ylabel('|F_y| / (mu_{base} F_z)');
title('Lateral utilization'); legend('Location','best');

nexttile; hold on;
for i=1:4
    plot(h.t,h.capacidad_x_rel(:,i),'-','Color',colores(i,:), ...
        'LineWidth',1.2,'DisplayName',[nombres{i} ' X']);
    plot(h.t,h.capacidad_y_rel(:,i),'--','Color',colores(i,:), ...
        'LineWidth',1.2,'DisplayName',[nombres{i} ' Y']);
end
marcasTiempo(h); grid on; xlabel('Time [s]'); ylabel('Capacity / (mu_{base} F_z)');
title('Residual capacity: X solid; Y dashed');
legend('Location','best','NumColumns',2); ylim([0 1.05]);
sgtitle([h.nombre ' | steering after 4 m of braking']);
end

function porRueda(t,valor,acel,freno,colores,nombres)
for i=1:4
    plot(t(acel),valor(acel,i),'-','Color',colores(i,:), ...
        'LineWidth',1.2,'DisplayName',nombres{i});
    plot(t(freno),valor(freno,i),'--','Color',colores(i,:), ...
        'LineWidth',1.2,'HandleVisibility','off');
end
end

function marcasTiempo(h)
if isfinite(h.tInicioFrenada)
    xline(h.tInicioFrenada,'k:','Braking','HandleVisibility','off');
end
if isfinite(h.tInicioGiro)
    xline(h.tInicioGiro,'m:','Turn','HandleVisibility','off');
end
end
