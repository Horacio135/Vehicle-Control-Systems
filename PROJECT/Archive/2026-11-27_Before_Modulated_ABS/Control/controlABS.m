function [FxCorregidas, info] = controlABS(FxSolicitadas, z, p)
% Limitador de frenada por rueda. Orden: [DI DD TI TD].
% Entradas: FxSolicitadas [N], estado físico completo z y parámetros p.
% Salidas: FxCorregidas (misma orientación) e información de intervención.
% Solo limita fuerzas negativas: conserva las demandas positivas y nulas.
% Emplea el 100 % de la capacidad efectiva actual del círculo de adherencia.
% No descuenta la fuerza lateral ni modifica el estado del modelo físico.
% Lee eta_brake del estado actual; la dinámica actualiza esa memoria una vez
% con la orden realmente aplicada. Es una aproximación por fuerza, sin
% velocidades de rueda ni regulación del deslizamiento longitudinal.
% Los nombres de parámetros y memorias compartidos se conservan para
% mantener la compatibilidad con pasoDinamico.

validateattributes(FxSolicitadas,{'numeric'},{'real','finite','vector','numel',4});
forma=size(FxSolicitadas);
solicitadas=double(FxSolicitadas(:).');
corregidas=solicitadas;
frenada=solicitadas<0;
info=struct('activo',any(frenada),'limitadas',false(forma), ...
    'Fzw',nan(forma),'Fmax_peak_w',nan(forma), ...
    'eta_brake',nan(forma),'Fmax_w',nan(forma));

if ~any(frenada)
    FxCorregidas=reshape(corregidas,forma);
    return
end

validateattributes(z,{'numeric'},{'real','finite','vector','numel',20});
z=z(:);

ax_anterior=z(7); ay_anterior=z(8);
dFzx=p.m*ax_anterior*p.hcg/p.L;
dFzyd=p.kRoll*p.m*ay_anterior*p.hcg/p.td;
dFzyt=(1-p.kRoll)*p.m*ay_anterior*p.hcg/p.tt;
Fzw=p.Fzw0+[-dFzx/2,-dFzx/2,dFzx/2,dFzx/2]+[-dFzyd,dFzyd,-dFzyt,dFzyt];
if any(~isfinite(Fzw)) || any(Fzw<0)
    error('controlABS:CargaVerticalInvalida', ...
        'Cargas verticales inválidas: el ABS no modifica el modelo físico.');
end
Fmax_peak_w=p.muw.*Fzw;

eta_brake=z(13:16).';
eta_brake=max(p.brake_slide_ratio,min(1,eta_brake));
Fmax_w=eta_brake.*Fmax_peak_w;
if any(~isfinite(Fmax_w)) || any(Fmax_w<0)
    error('controlABS:CapacidadInvalida','La capacidad de adherencia calculada debe ser finita y no negativa.');
end

corregidas(frenada)=-min(abs(solicitadas(frenada)),Fmax_w(frenada));
FxCorregidas=reshape(corregidas,forma);
info.Fzw=reshape(Fzw,forma);
info.Fmax_peak_w=reshape(Fmax_peak_w,forma);
info.eta_brake=reshape(eta_brake,forma);
info.Fmax_w=reshape(Fmax_w,forma);
info.limitadas=reshape(frenada & abs(solicitadas)>Fmax_w,forma);
end
