function [Fx,capacidad]=fuerzaLongitudinalPacejkaV2(kappa,Fz,mu,p)
% Fuerza longitudinal pura: tracción positiva y frenada negativa.
% Misma estructura de Magic Formula, con parámetros ajustables separados.
% No se introduce una pérdida adicional por bloqueo ni por tiempo.
tr=p.pacejka.longitudinal.traccion;fr=p.pacejka.longitudinal.frenada;
freno=kappa<0;
B=tr.B+freno.*(fr.B-tr.B);C=tr.C+freno.*(fr.C-tr.C);
E=tr.E+freno.*(fr.E-tr.E);factor=tr.factorMu+freno.*(fr.factorMu-tr.factorMu);
capacidad=mu.*Fz.*factor;q=B.*kappa;
Fx=capacidad.*sin(C.*atan(q-E.*(q-atan(q))));
end
