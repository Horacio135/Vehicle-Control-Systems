function z=inicializarEstadoPacejka(movimiento,p)
% Estado de 24 elementos: 1:8 movimiento/aceleraciones; 9:16 reservados=1;
% 17:20 bloqueo real diagnosticado; 21:24 omega [rad/s], orden DI DD TI TD.
movimiento=movimiento(:);
if numel(movimiento)==24,z=movimiento;return;end
validateattributes(movimiento,{'numeric'},{'real','finite','vector','numel',6});
vx=movimiento(4)-movimiento(6)*p.yw;
z=[movimiento;zeros(2,1);ones(8,1);zeros(4,1);max(0,vx(:))/p.pacejka.radio];
end
