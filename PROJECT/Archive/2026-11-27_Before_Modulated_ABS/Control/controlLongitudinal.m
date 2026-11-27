function [Fx, mem] = controlLongitudinal(est, ref, p, mem, dt)
% V12 longitudinal driver and PI, packaged as one callable block.
% est.vx: estimated speed [m/s]; ref.v_set: planned target speed [m/s].
% p: Kp_v, Ki_v, Fx_drive_max, Fx_brake_max and p.conductor settings.
% Output Fx is requested TOTAL longitudinal force [N], not per-wheel force.
% mem holds the PI integral, previous force and hysteretic driver mode.
if nargin<4 || isempty(mem), mem=struct('Fx',0,'modo',0,'e_int',0); end
[Fx,integral,driver]=controlConductorAgresivo( ...
    ref.v_set,est.vx,mem.e_int,dt,p,p.conductor,mem);
mem=driver;
mem.e_int=integral;
end

function [Fx,e_int,mem,objetivo]=controlConductorAgresivo(vRef,vx,e_int,dt,p,cfg,mem)
% The driver requests maximum force when speed error calls for driving/braking.
% The original PI is used only to maintain speed near the reference.
% Freeze its integral during full-demand interventions to avoid accumulating
% error that would artificially extend maximum application.
e=vRef-vx;
if e>cfg.errorActivar
    mem.modo=1;
elseif e < -cfg.errorActivar
    mem.modo=-1;
elseif (mem.modo==1 && e<=cfg.errorSoltar) || ...
        (mem.modo==-1 && e>=-cfg.errorSoltar)
    mem.modo=0;
end
if mem.modo==1
    objetivo=p.Fx_drive_max;
elseif mem.modo==-1
    objetivo=-p.Fx_brake_max;
else
    [objetivo,e_int]=controlVelocidadSimple(vRef,vx,e_int,dt,p);
end
% Fx is the continuous actuator output at the start of this step.
Fx=mem.Fx;
if objetivo < mem.Fx, tau=cfg.tauFreno; else, tau=cfg.tauAcel; end
mem.Fx=mem.Fx+(1-exp(-dt/tau))*(objetivo-mem.Fx);
end


function [Fx, e_int] = controlVelocidadSimple(v_ref, vx, e_int, dt, p)
    e = v_ref - vx;  e_int = e_int + e*dt;
    Fx = p.Kp_v*e + p.Ki_v*e_int;
    Fx = min(max(Fx, -p.Fx_brake_max), p.Fx_drive_max);
end

