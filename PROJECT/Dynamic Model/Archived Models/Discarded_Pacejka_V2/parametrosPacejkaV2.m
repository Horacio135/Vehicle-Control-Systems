function p=parametrosPacejkaV2(p)
% V2: Pacejka longitudinal/lateral y ponderaciones de deslizamiento combinado.
% No es una identificación MF6.2 completa. Parámetros iniciales editables.
% La estructura permite ajustar tracción y frenada independientemente.
forma=struct('B',10,'C',1.9,'E',.97,'factorMu',1);
p.pacejka.longitudinal.traccion=forma;
p.pacejka.longitudinal.frenada=forma;
p.pacejka.lateral=struct('C',1.3,'E',-.5,'factorMu',1);
p.pacejka.combinado=struct('B_xalpha',10,'C_xalpha',1,'E_xalpha',0, ...
    'B_ykappa',10,'C_ykappa',1,'E_ykappa',0);
% Ponderadores cosenoidales reducidos sin desplazamientos ni camber.
% Sus coeficientes son hipótesis editables; requieren datos de ensayo.
% B lateral se calcula de la rigidez con carga: B=Calpha/(C*mu_y*Fz).
% C y E laterales son hipótesis de forma, no valores medidos del vehículo.
p.pacejka.radio=.23;p.pacejka.inercia=.60;
p.pacejka.velocidadRegularizacion=.5;p.pacejka.pasoMaximo=.0002;
p.pacejka.potenciaMaxima=60000;
% Fuentes de formulación (no de un ajuste experimental a este vehículo):
% https://www.mathworks.com/help/sm/ref/magicformulatireforceandtorque.html
% https://www.mathworks.com/help/sdl/ref/tireroadinteractionmagicformula.html
end
