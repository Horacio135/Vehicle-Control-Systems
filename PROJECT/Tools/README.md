# Tools

`reproduce_run.m` vuelve a ejecutar una instantánea de [Run_Sources](../Results/Run_Sources/README.md) y guarda resultados a resolución completa. Requiere MATLAB; no requiere Python ni un vídeo previo.

```matlab
addpath('PROJECT/Tools');
outputFile = reproduce_run('V12_35_3D.m', fullfile(tempdir,'vehicle_v12'));
```

El ejemplo parte de la raíz del repositorio. Se desactiva el arranque automático de la animación en una copia temporal; las ecuaciones y los parámetros del ensayo no se cambian. Los Live Scripts originales y las instantáneas permanecen intactos. Se generan las gráficas del ensayo sin mostrarlas y se guarda un MAT completo; no se vuelve a renderizar un MP4. La copia temporal se conserva para trazabilidad.

Los archivos `verify_publication.py` y `publication_checks.json` comprueban el archivo publicado: enlaces Markdown locales, hashes, doce etapas, configuración de datos y conservación de las prácticas existentes. No son una validación experimental de la física.

[Registro de reproducción en MATLAB](matlab_reproduction_check.json): el caso V11 de 35 m/s se regeneró desde la instantánea organizada, con vuelta de 47.680 s y error máximo de 0.775982 m. Se comprobaron estados finitos, conservación de las fuentes y eliminación del temporizador. No se han vuelto a ejecutar todas las versiones históricas.
