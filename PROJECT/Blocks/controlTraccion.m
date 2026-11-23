function [FxRearCorrected, info] = controlTraccion(FxRearRequested, z, p, dt)
% Rear-wheel traction-demand limiter. Wheel order: [TI TD] = [rear left right].
% INPUTS: two requested forces [N], complete physical state z (20 elements),
%         plant parameters p, integration step dt [s].
% OUTPUT: two corrected requests, preserving row/column orientation.
% Only POSITIVE requests are limited; zero/braking requests pass unchanged.
% p.tc_factor optionally overrides the default 0.98 capacity multiplier.
% Capacity uses the SAME combined-force radius as pasoDinamico, not the
% longitudinal residual after reserving lateral force. No plant state is changed.

validateattributes(FxRearRequested,{'numeric'},{'real','finite','vector','numel',2});
shape=size(FxRearRequested);
requested=double(FxRearRequested(:).');
corrected=requested;
traction=requested>0;
info=struct('active',any(traction),'limited',false(shape), ...
    'capacityFactor',NaN,'FzwRear',nan(shape),'FmaxPeakRear',nan(shape), ...
    'etaBrakeRear',nan(shape),'FmaxRear',nan(shape),'limitRear',nan(shape));

% A pure braking/zero request bypasses TC, including capacity calculations.
if ~any(traction)
    FxRearCorrected=reshape(corrected,shape);
    return
end

validateattributes(z,{'numeric'},{'real','finite','vector','numel',20});
validateattributes(dt,{'numeric'},{'real','finite','scalar','positive'});
z=z(:);
capacityFactor=0.98;                 % Adjustable traction-demand fraction.
if isfield(p,'tc_factor'),capacityFactor=p.tc_factor;end
validateattributes(capacityFactor,{'numeric'},{'real','finite','scalar','nonnegative'});

% Exact V8 load-transfer expressions, evaluated using the same prior state.
ax_prev=z(7); ay_prev=z(8);
dFzx=p.m*ax_prev*p.hcg/p.L;
dFzyd=p.kRoll*p.m*ay_prev*p.hcg/p.td;
dFzyt=(1-p.kRoll)*p.m*ay_prev*p.hcg/p.tt;
Fzw=p.Fzw0+[-dFzx/2,-dFzx/2,dFzx/2,dFzx/2]+[-dFzyd,dFzyd,-dFzyt,dFzyt];
if any(~isfinite(Fzw)) || any(Fzw<0)
    error('controlTraccion:InvalidWheelLoad', ...
        'Invalid wheel loads: TC does not clip or repair the physical model.');
end
Fmax_peak_w=p.muw.*Fzw;

% In driving mode the plant releases braking on the driven wheels. Match its
% within-step eta_brake recovery and clamp, without writing the prediction to z.
eta_brake_prev=z(13:16).';
factor=1-exp(-dt/p.tau_brake_release);
eta_brake=eta_brake_prev+factor*(ones(1,4)-eta_brake_prev);
eta_brake=max(p.brake_slide_ratio,min(1,eta_brake));
Fmax_w=eta_brake.*Fmax_peak_w;
if any(~isfinite(Fmax_w)) || any(Fmax_w<0)
    error('controlTraccion:InvalidCapacity','The calculated adhesion capacity must be finite and nonnegative.');
end

limit=capacityFactor.*Fmax_w(3:4);
corrected(traction)=min(requested(traction),limit(traction));
FxRearCorrected=reshape(corrected,shape);
info.capacityFactor=capacityFactor;
info.FzwRear=reshape(Fzw(3:4),shape);
info.FmaxPeakRear=reshape(Fmax_peak_w(3:4),shape);
info.etaBrakeRear=reshape(eta_brake(3:4),shape);
info.FmaxRear=reshape(Fmax_w(3:4),shape);
info.limitRear=reshape(limit,shape);
info.limited=reshape(traction & requested>=limit,shape);
end
