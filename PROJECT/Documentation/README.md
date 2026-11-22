# Documentation — diseño y evolución V1–V12

Esta documentación reconstruye el proceso a partir del código, los artefactos guardados y las decisiones de la sesión de trabajo. No inventa fechas, mediciones ni commits intermedios. La fecha 2026-11-22 identifica la instantánea de publicación; las figuras y ejecuciones conservan configuraciones históricas.

- [Doce etapas y enlaces a sus archivos](Versions/README.md)
- [Decisiones transversales](Design_Decisions/README.md)
- [Contrato de la planta](../Dynamic%20Model/README.md)
- [Resultados y métricas](../Results/README.md)
- [Registro de archivos y hashes](artifact_manifest.json)
- [Ejecuciones y procedencia de datos](../Results/Numerical/README.md)

## Presentación intermedia

La [presentación intermedia](SCV_Evaluaci%C3%B3n_Intermedia_Proyecto_HHRB.pptx) se conserva sin editar. Expone el problema de subviraje/sobreviraje, la elección de ESC, su coordinación futura con ABS/TCS y la evolución de neumáticos y cargas. Sus 13 diapositivas representan un **hito intermedio**, no el estado final V12.

En ella, “ABS + TCS”, “ESC básico” e integración figuran pendientes. Las mejoras de estabilidad y rendimiento se describen como hipótesis a cuantificar; no deben leerse como resultados ya obtenidos por un ESC implementado. Los vídeos y tablas recientes amplían esa evidencia, pero siguen sin constituir validación experimental.

## Política del archivo

Se conservan nombres y bytes de fuentes, imágenes y vídeos seleccionados. Se excluyen autoguardados `.asv`, copias `.bak`, directorios `Respaldo_*`, un `untitled.m` y un duplicado de `Simulacion_skidpad`. No se publican credenciales, configuración de la máquina ni rutas absolutas del usuario en los manifiestos.

Los `.m` en Source Exports son representaciones de lectura de los MLX. Las instantáneas Run_Sources corresponden a configuraciones que produjeron resultados y pueden diferir de los archivos actuales. Las limitaciones de V7 y V9 se documentan en lugar de modificar las fuentes históricas. La planta permanece congelada para la siguiente fase de arquitectura por bloques.
