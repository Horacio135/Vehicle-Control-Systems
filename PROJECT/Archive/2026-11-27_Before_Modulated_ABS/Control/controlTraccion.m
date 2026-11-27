function [FxTraserasCorregidas, info] = controlTraccion(FxTraserasSolicitadas, z, p, dt)
% Limitador de tracción trasera. Orden de ruedas: [TI TD].
% Entradas: FxTraserasSolicitadas [N], estado físico completo z,
% parámetros p y paso de integración dt [s].
% Salidas: FxTraserasCorregidas (misma orientación) e información de control.
% Solo limita fuerzas positivas; conserva la frenada y las demandas nulas.
% factorCapacidad vale 0.98; p.tc_factor permite ajustar este valor.
% Usa el radio del círculo de adherencia, sin descontar la fuerza lateral.
% Predice la recuperación de eta_brake al soltar el freno durante dt,
% con las mismas expresiones del modelo, sin modificar el estado z.
% Los nombres de parámetros y memorias compartidos se conservan para
% mantener la compatibilidad con pasoDinamico.

validateattributes(FxTraserasSolicitadas,{'numeric'},{'real','finite','vector','numel',2});
forma=size(FxTraserasSolicitadas);
solicitadas=double(FxTraserasSolicitadas(:).');
corregidas=solicitadas;
traccion=solicitadas>0;
info=struct('activo',any(traccion),'limitadas',false(forma), ...
    'factorCapacidad',NaN,'FzTraseras',nan(forma),'FmaxPicoTraseras',nan(forma), ...
    'adherenciaFrenadoTraseras',nan(forma),'FmaxTraseras',nan(forma),'limiteTraseras',nan(forma));

if ~any(traccion)
    FxTraserasCorregidas=reshape(corregidas,forma);
    return
end

validateattributes(z,{'numeric'},{'real','finite','vector','numel',20});
validateattributes(dt,{'numeric'},{'real','finite','scalar','positive'});
z=z(:);
factorCapacidad=0.98;
if isfield(p,'tc_factor'),factorCapacidad=p.tc_factor;end
validateattributes(factorCapacidad,{'numeric'},{'real','finite','scalar','nonnegative'});

ax_anterior=z(7); ay_anterior=z(8);
dFzx=p.m*ax_anterior*p.hcg/p.L;
dFzyd=p.kRoll*p.m*ay_anterior*p.hcg/p.td;
dFzyt=(1-p.kRoll)*p.m*ay_anterior*p.hcg/p.tt;
Fzw=p.Fzw0+[-dFzx/2,-dFzx/2,dFzx/2,dFzx/2]+[-dFzyd,dFzyd,-dFzyt,dFzyt];
if any(~isfinite(Fzw)) || any(Fzw<0)
    error('controlTraccion:CargaVerticalInvalida', ...
        'Cargas verticales inválidas: el TC no modifica el modelo físico.');
end
Fmax_peak_w=p.muw.*Fzw;

adherenciaFrenadoAnterior=z(13:16).';
factorRecuperacion=1-exp(-dt/p.tau_brake_release);
eta_brake=adherenciaFrenadoAnterior+factorRecuperacion*(ones(1,4)-adherenciaFrenadoAnterior);
eta_brake=max(p.brake_slide_ratio,min(1,eta_brake));
Fmax_w=eta_brake.*Fmax_peak_w;
if any(~isfinite(Fmax_w)) || any(Fmax_w<0)
    error('controlTraccion:CapacidadInvalida','La capacidad de adherencia calculada debe ser finita y no negativa.');
end

limite=factorCapacidad.*Fmax_w(3:4);
corregidas(traccion)=min(solicitadas(traccion),limite(traccion));
FxTraserasCorregidas=reshape(corregidas,forma);
info.factorCapacidad=factorCapacidad;
info.FzTraseras=reshape(Fzw(3:4),forma);
info.FmaxPicoTraseras=reshape(Fmax_peak_w(3:4),forma);
info.adherenciaFrenadoTraseras=reshape(eta_brake(3:4),forma);
info.FmaxTraseras=reshape(Fmax_w(3:4),forma);
info.limiteTraseras=reshape(limite,forma);
info.limitadas=reshape(traccion & solicitadas>=limite,forma);
end
