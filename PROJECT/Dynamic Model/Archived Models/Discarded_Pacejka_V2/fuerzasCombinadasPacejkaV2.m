function [Fx,Fy,info]=fuerzasCombinadasPacejkaV2(kappa,alpha,Fz,mu,rigidez,p)
% Modelo reducido de Magic Formula combinada, sin camber ni desplazamientos.
% Fx=Gxalpha*FxPura; Fy=Gykappa*FyPura, con ponderadores cosenoidales.
% Así el patinamiento longitudinal también reduce la capacidad lateral,
% incluso cuando la fuerza longitudinal ha caído lejos de su máximo.
% Una elipse final protege la capacidad compartida de estos coeficientes
% no identificados. No se aplican eta_slip ni eta_brake adicionales.
[Fx0,Dx]=fuerzaLongitudinalPacejkaV2(kappa,Fz,mu,p);
[Fy0,Dy,By]=fuerzaLateralPacejkaV2(alpha,Fz,mu,rigidez,p);
c=p.pacejka.combinado;
q=c.B_xalpha.*alpha;
Gx=cos(c.C_xalpha.*atan(q-c.E_xalpha.*(q-atan(q))));
q=c.B_ykappa.*kappa;
Gy=cos(c.C_ykappa.*atan(q-c.E_ykappa.*(q-atan(q))));
Gx=max(0,min(1,Gx));Gy=max(0,min(1,Gy));
FxCombinada=Gx.*Fx0;FyCombinada=Gy.*Fy0;
uso=hypot(FxCombinada./max(Dx,eps),FyCombinada./max(Dy,eps));
factor=min(1,1./max(uso,eps));
Fx=factor.*FxCombinada;Fy=factor.*FyCombinada;
info=struct('FxPura',Fx0,'FyPura',Fy0,'capacidad_x',Dx, ...
    'capacidad_y',Dy,'B_lateral',By,'Gxalpha',Gx,'Gykappa',Gy, ...
    'factorCombinado',factor);
end
