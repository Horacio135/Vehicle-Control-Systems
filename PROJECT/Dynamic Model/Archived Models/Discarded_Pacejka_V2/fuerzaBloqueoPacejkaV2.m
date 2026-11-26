function [Fx,Fy]=fuerzaBloqueoPacejkaV2(vxLocal,vyLocal,Fz,mu,p)
% Modelo de deslizamiento completo para rueda bloqueada.
% Se usa la magnitud longitudinal Pacejka a kappa=-1 y fricción isotrópica
% opuesta a la velocidad de deslizamiento. No genera dirección al orientar
% la rueda; sí puede tener componente lateral si el coche se desplaza de lado.
[fuerza,~]=fuerzaLongitudinalPacejkaV2(-ones(size(Fz)),Fz,mu,p);
limiteLateral=mu.*Fz.*p.pacejka.lateral.factorMu;
modulo=min(abs(fuerza),limiteLateral);
velocidad=hypot(vxLocal,vyLocal);
Fx=-modulo.*vxLocal./max(velocidad,eps);
Fy=-modulo.*vyLocal./max(velocidad,eps);
end
