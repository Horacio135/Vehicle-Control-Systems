function [FxEmergency, deltaEmergency, active, mem] = controlEmergencia(est, obst, deltaRequested, p, mem)
% P5 TTC emergency braking with explicit force and steering outputs.
% est: X,Y,psi,vx estimate; obst: static obstacle X,Y.
% deltaRequested is the nominal steering command [rad]. P5 has no evasive
% steering controller, so this block preserves that angle in both modes.
% FxEmergency is zero when inactive; apply it ONLY when active is true.
% p: ttc_umbral [s], Fx_brake_max [N]. Carry mem between calls.
if nargin<5 || isempty(mem), mem=struct('latched',false); end
dx=obst.X-est.X;
dy=obst.Y-est.Y;
range=hypot(dx,dy);
ahead=(cos(est.psi)*dx+sin(est.psi)*dy)>0;
closingSpeed=est.vx;
if ahead && closingSpeed>0.1
    ttc=range/closingSpeed;
else
    ttc=inf;
end
trigger=ahead && (ttc<p.ttc_umbral);
mem.latched=mem.latched || trigger;
active=mem.latched;
FxEmergency=0;
if active, FxEmergency=-p.Fx_brake_max; end
deltaEmergency=deltaRequested;
end
