%% Modular project - first stage: standalone dynamic plant
% Run this script. Edit the scenario settings, initial state and p below.
% Only the dynamic block is connected at this stage. Sensor, estimator,
% combined motion control, emergency, TC and ABS blocks are not implemented here.
templateDirectory = fileparts(mfilename('fullpath'));
blocksDirectory = fullfile(fileparts(templateDirectory),'Blocks');
addpath(blocksDirectory);
plant = @pasoDinamico;
plantInfo = functions(plant);
assert(strcmpi(plantInfo.file,fullfile(blocksDirectory,'pasoDinamico.m')), ...
    'Another pasoDinamico shadows this block. Set Current Folder to the Blocks directory.');

%% Simulation settings and initial motion state
cfg.dt = 0.002;                      % Integration step [s]
cfg.maxTime = 20;                    % Scenario time limit [s]
cfg.targetSpeed = 35;                % Start braking at this vx [m/s]
cfg.stopSpeed = 0.01;                % End braking below this vx [m/s]
cfg.steeringAngle = 0;               % Front-wheel angle [rad]
cfg.initialMotion = [0;0;0;0;0;0];  % [X;Y;psi;vx;vy;r]
cfg.makePlots = true;

%% Reference V8 vehicle parameters - unchanged
p = struct(); 

p.m    = 230;      % mass [kg]
p.g    = 9.81;     % gravity [m/s^2]
p.rho  = 1.225;    % air density [kg/m^3]
p.Cd   = 1.0;      % aerodynamic coefficient
p.Af   = 1.1;      % frontal area [m^2]
p.fr  = 0.022;    % rolling-resistance coefficient
p.Fx_drive_max = 2500;   % maximum driving force [N]
p.Fx_brake_max = 5000;   % maximum braking force [N]
p.theta = 0;      % road inclination [rad]

% Define the parameters needed to describe the vehicle's
% lateral dynamics here.
p.ld = 0.765; % [m] CG to front axle, longitudinal distance
p.lt = 0.765; % [m] CG to rear axle, longitudinal distance
p.lyd = 0.600; % [m] CG to front wheels, lateral distance
p.lyt = 0.600; % [m] CG to rear wheels, lateral distance
p.hcg = 0.230; % [m] centre-of-gravity height
% Wheel coordinates relative to the CG
% Order: DI, DD, TI, TD
p.xw = [p.ld,p.ld,-p.lt,-p.lt];
p.L = p.ld+p.lt; 
p.yw = [p.lyd, -p.lyd, p.lyt, -p.lyt];
p.td = p.lyd + p.lyt;
p.tt = p.td; 
p.Iz = 130; % [kg m^2] yaw inertia
p.Cad = 35000; % [N/rad] front cornering stiffness
p.Cat = 50000; % [N/rad] rear cornering stiffness --> understeering vehicle
p.Caw = [p.Cad/2, p.Cad/2, p.Cat/2, p.Cat/2];
p.nCa = 0.8; 
p.mu_t = 1.5; % rear tyre friction
p.mu_d = 0.95*p.mu_t; % front tyre friction
p.muw = [p.mu_d, p.mu_d, p.mu_t, p.mu_t];
% Lateral adhesion loss under excess traction demand
p.Kslip = 3; % Rear-wheel lateral-loss intensity
% Time dynamics of lateral loss and recovery
p.tau_slip_loss = 0.03; % loss time constant [s]
p.tau_slip_rec  = 0.50; % recovery time constant [s]
% Adhesion loss under excess braking demand.
% Initial test values, awaiting calibration.
% Two braking modes: normal and approximate lock.
% Illustrative parameters awaiting experimental calibration.
p.brake_slide_ratio = 0.68; % mu_min / mu_base (high-speed limit)
p.v_brake_slide = 15;      % Speed scale [m/s]
p.brake_lock_on = 1.02;    % Entry: demand / nominal capacity
p.brake_lock_off = 0.90;   % Hysteretic exit
p.tau_brake_loss = 0.03;   % Transition to lock-mode target [s]
p.tau_brake_rec = 0.10;    % Recovery under partial brake relief [s]
p.tau_brake_release = 0.01;% Rapid full-release recovery: 95% in 30 ms

% Force allocation
p.Fzd0 = p.m*p.g*p.lt/p.L; % [N] initial front-axle vertical load
p.Fzt0 = p.m*p.g*p.ld/p.L; % [N] initial rear-axle vertical load
p.Fzw0 = [p.Fzd0/2, p.Fzd0/2, p.Fzt0/2, p.Fzt0/2]; % [N] initial vertical load per wheel
p.drive_w = [0, 0, 0.5, 0.5]; % Driving-force allocation during acceleration
p.brake_w = [0.3, 0.3, 0.2, 0.2]; % Longitudinal-force allocation during braking
p.kRoll = 0.5; % front share of roll moment (replace with a suspension model later)

%% Settings for the separated blocks (not connected in this plant-only test)
p.dt = cfg.dt;
p.Kp_v = 800; p.Ki_v = 200;
p.Ld0 = 2.0; p.kLd = 0.3; p.delta_max = 0.5;
p.conductor.tauAcel = 0.010; p.conductor.tauFreno = 0.020;
p.conductor.errorActivar = 0.35; p.conductor.errorSoltar = 0.05;
p.gps_sigma = 0.5; p.acc_sigma = 0.2; p.gyro_sigma = 0.01;
p.gps_cada = 25; p.gps_corte = []; p.gyro_bias = 0;
p.Q = diag([0.02 0.02 0.005 0.10].^2);
p.R = diag([p.gps_sigma p.gps_sigma].^2);
p.ttc_umbral = 2.0;


%% Initialize the complete physical state and storage
validateattributes(cfg.dt,{'numeric'},{'scalar','real','finite','positive'});
validateattributes(cfg.maxTime,{'numeric'},{'scalar','real','finite','positive'});
z = inicializarEstadoDinamico(cfg.initialMotion);
maxSteps = ceil(cfg.maxTime/cfg.dt);
history = struct();
history.z = zeros(maxSteps+1,numel(z));
history.z(1,:) = z.';
history.u = zeros(maxSteps,2);
history.tele = cell(maxSteps,1);
history.braking = false(maxSteps,1);
history.brakeStartTime = NaN;
history.stopReason = "time limit";
braking = false;
completedSteps = 0;

%% Scenario -> dynamic block -> next state and telemetry
for k = 1:maxSteps
    t = (k-1)*cfg.dt;

    % Scenario event logic, not TC, ABS or a speed controller.
    % Direct use of true vx is confined to scheduling this plant-only test.
    if ~braking && z(4) >= cfg.targetSpeed
        braking = true;
        history.brakeStartTime = t;
    end
    if braking && z(4) <= cfg.stopSpeed
        history.stopReason = "stopping threshold reached";
        break
    end
    if braking
        requestedFx = -p.Fx_brake_max;
    else
        requestedFx = p.Fx_drive_max;
    end
    u = [requestedFx; cfg.steeringAngle];

    % Future insertion point: sensors -> estimator -> combined motion control
    % -> emergency arbitration -> TC/ABS -> u. No identity/mock blocks are used.
    [z_next, tele] = plant(z, u, p, cfg.dt);

    % Reject an invalid run instead of altering the frozen plant equations.
    if ~isreal(z_next) || any(~isfinite(z_next)) || ...
            ~isreal(tele.Fzw) || any(tele.Fzw < 0)
        history.stopReason = "invalid state or negative wheel load";
        warning('runSimulation:InvalidStep', ...
            'Stopped at t=%.3f s; no physical values were clipped by the harness.',t);
        break
    end
    history.u(k,:) = u.';
    history.tele{k} = tele;
    history.braking(k) = braking;
    history.z(k+1,:) = z_next.';
    z = z_next;                       % Carry all 20 components into the next call.
    completedSteps = k;
end

%% Align states, inputs and telemetry in time
% z(k,:) is the beginning of step k; u(k,:) and tele{k} belong to that step.
% z(k+1,:) is the next state. Therefore z has one more row than u and tele.
history.z = history.z(1:completedSteps+1,:);
history.u = history.u(1:completedSteps,:);
history.tele = history.tele(1:completedSteps);
history.braking = history.braking(1:completedSteps);
history.t = (0:completedSteps).'*cfg.dt;
history.tInput = history.t(1:end-1);
history.distance = [0;cumsum(vecnorm(diff(history.z(:,1:2),1,1),2,2))];
history.FxTransmittedChassis = cellfun(@(te)sum(te.Fxw_chasis),history.tele);
fprintf('Standalone plant: %s, %.3f s, peak vx %.3f m/s, distance %.3f m.\n', ...
    history.stopReason,history.t(end),max(history.z(:,4)),history.distance(end));

%% Basic results - no animation or video is started automatically
if cfg.makePlots && completedSteps > 0
    figure('Name','Standalone V8 plant - modular template','Color','w');
    tiledlayout(2,2,'TileSpacing','compact');
    nexttile;plot(history.t,history.z(:,4),'LineWidth',1.3);grid on;
    xlabel('Time [s]');ylabel('vx [m/s]');title('Longitudinal speed');
    nexttile;plot(history.z(:,1),history.z(:,2),'LineWidth',1.3);grid on;axis equal;
    xlabel('X [m]');ylabel('Y [m]');title('CG trajectory');
    nexttile;plot(history.tInput,history.u(:,1),'k--', ...
        history.tInput,history.FxTransmittedChassis,'b-','LineWidth',1.2);grid on;
    xlabel('Time [s]');ylabel('Force [N]');title('Requested and transmitted force');
    legend('Total request','Sum of chassis-frame tyre Fx','Location','best');
    nexttile;plot(history.t,history.distance,'LineWidth',1.3);grid on;
    xlabel('Time [s]');ylabel('Distance [m]');title('Travelled distance');
end
