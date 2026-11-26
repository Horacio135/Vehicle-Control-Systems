function [Fy,capacidad,B]=fuerzaLateralPacejkaV2(alpha,Fz,mu,rigidez,p)
% Magic Formula lateral pura, alpha en radianes con signo restaurador.
% B*C*D=rigidez conserva exactamente la pendiente original en alpha=0.
% Curva con crecimiento, pico y caída postpico; sin tanh posterior.
forma=p.pacejka.lateral;
capacidad=mu.*Fz.*forma.factorMu;
B=rigidez./max(forma.C.*capacidad,eps);q=B.*alpha;
Fy=capacidad.*sin(forma.C.*atan(q-forma.E.*(q-atan(q))));
end
