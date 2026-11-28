function Verificar_videos_telemetria_ESC(origen,salida)
% Verifica los MP4 y registra los datos utilizados sin modificarlos.
f=fopen(origen,'r');assert(f>=0);limpiar=onCleanup(@()fclose(f));b=fread(f,Inf,'*uint8');
md=java.security.MessageDigest.getInstance('SHA-256');md.update(b);
hash=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));clear limpiar b;
informe=struct('fecha','2026-11-28','origen_sha256',hash, ...
 'simulacion_recalculada',false,'reproduccion',1,'dt_datos_s',.002, ...
 'ayudas',{{'ABS','TC','ESC'}},'sensores_estimador',false);
tipos={'2D','3D'};
for j=1:2
 nombre=['ESC_ABS_TC_' tipos{j} '_telemetria_x1.mp4'];r=VideoReader(fullfile(salida,nombre));
 assert(r.Duration>190&&r.Duration<191&&r.FrameRate==20);
 for t=[0 95 189.9],r.CurrentTime=t;assert(~isempty(readFrame(r)));end
 r.CurrentTime=100;imwrite(readFrame(r),fullfile(salida,['Verificacion_mitad_' tipos{j} '.png']));
 informe.videos(j)=struct('archivo',nombre,'duracion_s',r.Duration,'fps',r.FrameRate,'ancho',r.Width,'alto',r.Height,'lectura_verificada',true);
end
f=fopen(fullfile(salida,'Verificacion_videos.json'),'w','n','UTF-8');fprintf(f,'%s',jsonencode(informe,PrettyPrint=true));fclose(f);disp(informe);
end
