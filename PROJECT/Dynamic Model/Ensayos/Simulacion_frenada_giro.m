function [histLeve, histAcusado] = Simulacion_frenada_giro(p, dt, fnDinamico)
% Acelerar a 35 m/s, frenar al maximo y dirigir tras 4 m de frenada.
% La direccion representa el angulo de rueda, no el del volante.
% Usa pasoDinamico(z,u,p,dt) con estado completo de 20 componentes.
% No alivia automaticamente el freno ni modifica los parametros fisicos.
% Uso desde el principal:
% [histGiroLeve,histGiroAcusado] = Simulacion_frenada_giro(p,dt,@pasoDinamico);

validateattributes(dt,{'numeric'},{'scalar','real','finite','positive'});
assert(isa(fnDinamico,'function_handle'),'Se requiere @pasoDinamico.');
config.vObjetivo = 35;             % [m/s]
config.distanciaGiro = 4;           % Desde inicio de frenada [m]
config.tEntrada = 0.15;             % Entrada suave de direccion [s]
config.Tmax = 120;                 % Limite temporal [s]
config.vParada = 0.01;             % Umbral de parada [m/s]

histLeve = ejecutarCaso(p,dt,fnDinamico,config,deg2rad(2),'Giro leve (2 grados)');
histAcusado = ejecutarCaso(p,dt,fnDinamico,config,deg2rad(10),'Giro acusado (10 grados)');
mostrarCaso(histLeve,p);
mostrarCaso(histAcusado,p);

figure('Name','Frenada con giro - comparacion','Color','w');
subplot(2,1,1); hold on;
plot(histLeve.z(:,1),histLeve.z(:,2),'b-','LineWidth',1.4);
plot(histAcusado.z(:,1),histAcusado.z(:,2),'r--','LineWidth',1.4);
axis equal; grid on; xlabel('X [m]'); ylabel('Y [m]');
title('Trayectorias: el bloqueo puede impedir ambos giros');
legend('Leve','Acusado','Location','best');
subplot(2,1,2); hold on;
plot(histLeve.t,rad2deg(histLeve.delta),'b-','LineWidth',1.4);
plot(histAcusado.t,rad2deg(histAcusado.delta),'r--','LineWidth',1.4);
grid on; xlabel('Tiempo [s]'); ylabel('Direccion de rueda [grados]');
title('Direccion solicitada despues de 4 m de frenada');
legend('Leve','Acusado','Location','best');
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
    % Longitud acumulada de la trayectoria, no distancia al origen.
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
% Capacidad residual para una componente, manteniendo la otra fuerza actual.
% No sumar ambas: comparten el mismo circulo de adherencia.
h.capacidad_x=sqrt(max(h.Fmax_w.^2-h.Fyw.^2,0)).*h.factor_x_bloqueo;
h.capacidad_y=sqrt(max(h.Fmax_w.^2-h.Fxw.^2,0)).* ...
    h.factor_y_bloqueo.*h.eta_slip;
h.capacidad_x_rel=h.capacidad_x./nominal;
h.capacidad_y_rel=h.capacidad_y./nominal;
if ~h.paradaAlcanzada, warning('%s: ensayo terminado sin alcanzar parada.',nombre); end
if isnan(tGiro), warning('%s: no se alcanzaron 4 m de frenada.',nombre); end
fprintf('\n--- %s ---\n',nombre);
fprintf('Aceleracion: %.3f s; frenada: %.3f s; distancia frenada: %.3f m\n', ...
    tFrenada,h.t(end)-tFrenada,h.distancia(end)-sFrenada);
fprintf('Inicio del giro: %.3f m despues de empezar a frenar\n',sGiro-sFrenada);
fprintf('Maxima fuerza lateral absoluta por rueda [N]: ');
fprintf('%.3f ',max(abs(h.Fyw),[],1)); fprintf('\n');
end

function mostrarCaso(h,p)
colores=lines(4); nombres={'DI','DD','TI','TD'};
acel=~h.frenando; freno=h.frenando;
idx=find(freno,1); if ~isempty(idx), acel(idx)=true; end
figure('Name',['Frenada con giro - ' h.nombre],'Color','w', ...
    'Position',[80 80 1400 850]);
tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

nexttile; hold on;
plot(h.t(acel),h.z(acel,4),'b-','LineWidth',1.4,'DisplayName','Aceleracion');
plot(h.t(freno),h.z(freno,4),'b--','LineWidth',1.4,'DisplayName','Frenada');
marcasTiempo(h); grid on; xlabel('Tiempo [s]'); ylabel('v_x [m/s]');
title('Velocidad / tiempo'); legend('Location','best');

nexttile; hold on;
plot(h.distancia(acel),h.z(acel,4),'b-','LineWidth',1.4);
plot(h.distancia(freno),h.z(freno,4),'b--','LineWidth',1.4);
if isfinite(h.sInicioFrenada), xline(h.sInicioFrenada,'k:','Frenada'); end
if isfinite(h.sInicioGiro), xline(h.sInicioGiro,'m:','Giro'); end
grid on; xlabel('Distancia acumulada [m]'); ylabel('v_x [m/s]');
title('Velocidad / metros');

nexttile; hold on;
plot(h.t(acel),h.distancia(acel),'b-','LineWidth',1.4);
plot(h.t(freno),h.distancia(freno),'b--','LineWidth',1.4);
marcasTiempo(h); grid on; xlabel('Tiempo [s]'); ylabel('Distancia acumulada [m]');
title('Distancia / tiempo');

nexttile; hold on;
porRueda(h.t,h.aprov_long,acel,freno,colores,nombres);
yline(1,'k:','Limite nominal','HandleVisibility','off');
marcasTiempo(h); grid on; xlabel('Tiempo [s]'); ylabel('|F_x| / (mu_{base} F_z)');
title('Aprovechamiento longitudinal'); legend('Location','best');

nexttile; hold on;
porRueda(h.t,h.aprov_lat,acel,freno,colores,nombres);
yline(1,'k:','Limite nominal','HandleVisibility','off');
marcasTiempo(h); grid on; xlabel('Tiempo [s]'); ylabel('|F_y| / (mu_{base} F_z)');
title('Aprovechamiento lateral'); legend('Location','best');

nexttile; hold on;
for i=1:4
    plot(h.t,h.capacidad_x_rel(:,i),'-','Color',colores(i,:), ...
        'LineWidth',1.2,'DisplayName',[nombres{i} ' X']);
    plot(h.t,h.capacidad_y_rel(:,i),'--','Color',colores(i,:), ...
        'LineWidth',1.2,'DisplayName',[nombres{i} ' Y']);
end
marcasTiempo(h); grid on; xlabel('Tiempo [s]'); ylabel('Capacidad / (mu_{base} F_z)');
title('Capacidad residual: X continua; Y discontinua');
legend('Location','best','NumColumns',2); ylim([0 1.05]);
sgtitle([h.nombre ' | direccion tras 4 m de frenada']);
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
    xline(h.tInicioFrenada,'k:','Frenada','HandleVisibility','off');
end
if isfinite(h.tInicioGiro)
    xline(h.tInicioGiro,'m:','Giro','HandleVisibility','off');
end
end
