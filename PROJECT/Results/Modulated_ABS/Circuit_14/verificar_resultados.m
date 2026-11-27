base=fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
salida=fileparts(mfilename('fullpath'));
d=load(fullfile(salida,'lazo_completo.mat'));
assert(~d.pSinABS.abs.habilitado && d.pConABS.abs.habilitado);
assert(strcmp(d.sinControl.motivo,'Vuelta completada') && strcmp(d.conControl.motivo,'Vuelta completada'));
assert(any(d.sinControl.modos==1) && any(d.conControl.modos==1),'TC debe intervenir en ambos casos');
assert(~any(d.sinControl.modos==2) && any(d.conControl.modos==2),'Conmutación ABS incorrecta');
nombres={'Comparacion_ABS_off_on_TC_on_2D_x4.mp4','ABS_TC_on_real_estimado_2D.mp4'};
for j=1:2
 v=VideoReader(fullfile(salida,nombres{j}));
 for fraccion=[0 .5 .99]
  v.CurrentTime=fraccion*(v.Duration-1/v.FrameRate); fotograma=readFrame(v);
  assert(~isempty(fotograma));
 end
 fprintf('Decodificado inicio/medio/final: %s (%dx%d, %.2f s)\n',nombres{j},v.Width,v.Height,v.Duration);
end
fprintf('TC activo en ambos; ABS solo en caso activado; ambas vueltas completadas.\n');
