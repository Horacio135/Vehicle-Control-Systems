function p=parametrosSoloTraccion(p)
% Parámetros iniciales de estudio; no identificados con ensayos del vehículo.
% Forma longitudinal típica de asfalto seco publicada por MathWorks:
% https://www.mathworks.com/help/sdl/ref/tireroadinteractionmagicformula.html
% El pico D se escala con p.muw, conservando la adherencia base del proyecto.
p.pacejka.B=10;p.pacejka.C=1.9;p.pacejka.E=.97;
p.pacejka.radio=.23;                % Radio efectivo de rueda [m], supuesto.
p.pacejka.inercia=.60;              % Inercia por rueda [kg m^2], supuesta.
p.pacejka.velocidadRegularizacion=.5; % Denominador regularizado [m/s].
p.pacejka.pasoMaximo=.0002;         % Subpaso para integrar la rueda [s].
p.pacejka.potenciaMaxima=60000;     % Potencia mecánica total [W], supuesta.
% Inf desactiva el límite de potencia. No se fuerza recuperación con v.
end
