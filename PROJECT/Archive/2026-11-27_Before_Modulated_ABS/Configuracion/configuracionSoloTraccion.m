function [p,cfg,plan]=configuracionSoloTraccion
% Parámetros editables de la comparación. Sensores y estimador omitidos.
p = struct(); 

p.m    = 230;
p.g    = 9.81;
p.rho  = 1.225;
p.Cd   = 1.0;
p.Af   = 1.1;
p.fr  = 0.022;
p.Fx_drive_max = 5000;
p.Fx_brake_max = 5000;
p.theta = 0;

p.ld = 0.765;
p.lt = 0.765;
p.lyd = 0.600;
p.lyt = 0.600;
p.hcg = 0.230;
p.xw = [p.ld,p.ld,-p.lt,-p.lt];
p.L = p.ld+p.lt; 
p.yw = [p.lyd, -p.lyd, p.lyt, -p.lyt];
p.td = p.lyd + p.lyt;
p.tt = p.td; 
p.Iz = 130;
p.Cad = 35000;
p.Cat = 50000;
p.Caw = [p.Cad/2, p.Cad/2, p.Cat/2, p.Cat/2];
p.nCa = 0.8; 
p.mu_t = 1.5;
p.mu_d = 0.95*p.mu_t;
p.muw = [p.mu_d, p.mu_d, p.mu_t, p.mu_t];
p.Kslip = 3;
p.tau_slip_loss = 0.03;
p.tau_slip_rec  = 0.50;
p.brake_slide_ratio = 0.68;
p.v_brake_slide = 15;
p.brake_lock_on = 1.02;
p.brake_lock_off = 0.90;
p.tau_brake_loss = 0.03;
p.tau_brake_rec = 0.10;
p.tau_brake_release = 0.01;

p.Fzd0 = p.m*p.g*p.lt/p.L;
p.Fzt0 = p.m*p.g*p.ld/p.L;
p.Fzw0 = [p.Fzd0/2, p.Fzd0/2, p.Fzt0/2, p.Fzt0/2];
p.drive_w = [0, 0, 0.5, 0.5];
p.brake_w = [0.3, 0.3, 0.2, 0.2];
p.kRoll = 0.5;



%% Configuración editable
p.Kp_v=800;p.Ki_v=200;p.Ld0=1;p.kLd=.15;p.delta_max=.5;
p.dt=.002;p.tc_factor=.98;p.ttc_umbral=2;
p.conductor=struct('tauAcel',.010,'tauFreno',.020,'errorActivar',.35,'errorSoltar',.05);
cfg=struct('ancho',3,'pasoTrayectoria',.25,'pasoConos',3, ...
    'velocidadMaxima',35,'tiempoMaximo',180,'errorAbandono',10);
% [] desactiva nuevas detecciones. Para probar emergencia: struct('X',100,'Y',0).
cfg.obstaculo=[];
plan=struct('ayMax',11,'axAcel',6,'axFreno',12,'anticipacionPI',0);

p=parametrosSoloTraccion(p);
end
