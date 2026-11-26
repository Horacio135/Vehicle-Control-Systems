function fuerza=fuerzaPacejka(deslizamiento,carga,mu,p)
% Magic Formula longitudinal pura. Deslizamiento con signo, sin recortarlo
% a [-1,1]: durante tracción la definición (R*omega-v)/max(|v|,v0) supera 1.
% No hay multiplicadores eta_slip ni eta_brake sobre esta fuerza.
q=p.pacejka.B.*deslizamiento;
fuerza=mu.*carga.*sin(p.pacejka.C.*atan(q-p.pacejka.E.*(q-atan(q))));
end
