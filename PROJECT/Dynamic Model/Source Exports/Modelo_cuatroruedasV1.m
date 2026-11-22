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
% Wheel coordinates relative to the CG
% Order: DI, DD, TI, TD
p.xw = [p.ld,p.ld,-p.lt,-p.lt];
p.yw = [p.lyd, -p.lyd, p.lyt, -p.lyt];
p.Iz = 130; % [kg m^2] yaw inertia
p.Cad = 35000; % [N/rad] front cornering stiffness
p.Cat = 50000; % [N/rad] rear cornering stiffness --> understeering vehicle
p.Caw = [p.Cad/2, p.Cad/2, p.Cat/2, p.Cat/2];
p.mu_t = 1.5; % rear tyre friction
p.mu_d = 0.95*p.mu_t; % front tyre friction
p.muw = [p.mu_d, p.mu_d, p.mu_t, p.mu_t];

% Longitudinal force allocation
p.drive_w = [0.25,0.25,0.25,0.25];
p.brake_w = [0.3,0.3,0.2,0.2];



% Longitudinal and lateral dynamics integration

% u: control variable = [Fx; delta]
% z: vehicle state vector [X; Y; psi; vx; vy; r]
% X, Y = global CG position
% psi = heading / yaw angle
% vx, vy = chassis-frame velocities
% r = yaw rate

z0 = [0; -geom.R; 0; ctrl.v_skid; 0; 0]; % initial race-car state

function z_next = pasoCinematico(z, u, p, dt)
    Fx = u(1);
    delta = u(2);
    X = z(1); 
    Y = z(2);
    psi = z(3);
    vx = z(4);
    L = p.ld + p.lt;

    % Longitudinal dynamics (P1)
    Ra = 0.5*p.rho*p.Cd*p.Af*vx^2; 
    Rr = p.fr*p.m*p.g*cos(p.theta);
    Rg = p.m*p.g*sin(p.theta);
    ax = (Fx - Ra - sign(vx)*Rr - Rg)/p.m; % longitudinal equation of motion
    vx_next = max(vx + ax*dt,0);

    %% Lateral dynamics [kinematic]
    beta = atan((p.lt/L)*tan(delta)); % kinematic sideslip angle
    r = (vx/p.lt)*tan(beta);
    vy = vx*tan(beta);

    % transform to global coordinates
    X_next = X + (vx*cos(psi)-vy*sin(psi))*dt;
    Y_next = Y + (vx*sin(psi)+vy*cos(psi))*dt;
    psi_next = psi + r*dt; 

    z_next = [X_next; Y_next; psi_next; vx_next; vy; r]; % state vector at the next time step

end

function z_next = pasoDinamico(z, u, p, dt)
    Fx = u(1);
    delta = u(2);
    X = z(1); 
    Y = z(2);
    psi = z(3);
    vx = z(4);
    vy = z(5);
    r = z(6);
    L = p.ld + p.lt;
    xw = p.xw;
    yw = p.yw;


    % vx_c = max(vx, 1.0); % corrected speed to avoid division by zero,
    % disabled now that the model uses atan2

    % Local velocities of the four wheels
    vxw = vx - r.*yw;
    vyw = vy + r.*xw;

    % per-wheel slip angles and lateral forces
    delta_w = [delta, delta, 0, 0];
    alpha_w = delta_w - atan2(vyw,vxw);
    
    % Lateral forces per wheel
    Fyw = p.Caw.*alpha_w;

    %% Define adhesion limits
    Fzd = p.m*p.g*p.lt/L; 
    Fzt = p.m*p.g*p.ld/L; 
    Fzw = [Fzd/2, Fzd/2, Fzt/2, Fzt/2]; % vertical force for each wheel DI, DD, TI, TD

    % Individual lateral saturation of all four wheels
    Fymax_w = p.muw.*Fzw;
    Fyw = max(min(Fyw, Fymax_w), -Fymax_w); 

    % Longitudinal grip
    % Longitudinal allocation per wheel

    if Fx >= 0
        Fxw = Fx .* p.drive_w;
    else
        Fxw = Fx .* p.brake_w;
    end

    % Individual longitudinal saturation
    Fxmax_w = p.muw .* Fzw;
    Fxw = max(min(Fxw, Fxmax_w), -Fxmax_w);
    
    % Chassis-frame components of lateral forces
    Fxw_chasis = Fxw .* cos(delta_w) - Fyw .* sin(delta_w);

    Fyw_chasis = Fxw .* sin(delta_w) + Fyw .* cos(delta_w);

    % Resultant forces and moment
    SumFx = sum(Fxw_chasis); % Total longitudinal force
    SumFy = sum(Fyw_chasis); % Total lateral force
    SumMz = sum(p.xw .* Fyw_chasis - p.yw .* Fxw_chasis); % Yaw moment about the CG

    % Longitudinal dynamics (P1)
    Ra = 0.5*p.rho*p.Cd*p.Af*vx^2; 
    Rr = p.fr*p.m*p.g*cos(p.theta);
    Rg = p.m*p.g*sin(p.theta);
    
    % equations of motion
    ax = (SumFx - Ra - sign(vx)*Rr - Rg)/p.m + vy*r; % retain centripetal coupling
    ay = SumFy/p.m - vx*r; 
    dr = SumMz/p.Iz;   
   
    vx_next = max(vx + ax*dt,0);
    vy_next = vy + ay*dt; 
    r_next = r + dr*dt;
    X_next = X + (vx*cos(psi)-vy*sin(psi))*dt;
    Y_next = Y + (vx*sin(psi)+vy*cos(psi))*dt;
    psi_next = psi + r*dt; 

    z_next = [X_next; Y_next; psi_next; vx_next; vy_next; r_next]; 
end