# Decisiones de diseño

## Del movimiento del chasis al contacto por rueda

Se mantuvo un chasis plano con X, Y y guiñada y sus velocidades para poder comparar un modelo cinemático con otro dinámico. Calcular fuerzas, cargas y momentos por rueda permitió preparar el frenado diferencial futuro sin confundir cuatro contactos con cuatro velocidades angulares simuladas. La telemetría se añadió para observar la saturación que un único estado de posición ocultaría.

## Sustituir límites independientes por adherencia combinada

Limitar Fx y Fy por separado permite gastar el mismo presupuesto de adherencia dos veces. V3 aplica un círculo compartido. La demanda de fuerza y la fuerza transmitida se guardan como conceptos distintos: subir la primera no garantiza subir la segunda. V4 añadió transferencia cuasiestática y rigidez dependiente de Fz para que cada rueda tenga una capacidad distinta durante las maniobras.

## Exceso de acelerador y pérdida lateral

V5 añadió pérdida de capacidad lateral en las ruedas traseras motrices cuando la demanda longitudinal supera la capacidad disponible. Se usaron memorias y transiciones exponenciales para representar pérdida rápida y recuperación más lenta, sin introducir velocidad angular ni pares de rueda. V6 pasó a trabajar con las velocidades proyectadas en los ejes de cada rueda. Esta simplificación permite ensayar inestabilidad, pero no equivale a un neumático calibrado ni a un TCS físico.

## Frenado: saturación, bloqueo aproximado y recuperación

La primera recta de aceleración/frenado mostró que la saturación sola podía producir una deceleración casi constante. Una curva de velocidad más curvada no demuestra por sí sola realismo: con fuerza aproximadamente constante, una frenada lineal puede tener sentido. El límite tampoco es siempre g; depende de mu, cargas y resistencias.

Para estudiar pérdida de eficacia bajo una demanda excesiva se acordó un segundo modo, disparado por demanda/capacidad nominal y separado del funcionamiento normal mediante histéresis. La capacidad en ese modo cae con la velocidad a través de una exponencial con límite inferior. Se reescribe como `mu_eff=mu_base-[mu_base*(1-eta)]`, equivalente a restar una pérdida creciente que se satura. No se adoptó como una ley universal del neumático.

Se distinguió aliviar de **soltar completamente** el freno. Al soltar, la demanda de frenado es cero, sale el modo y la memoria recupera rápidamente la capacidad. Con tau=0.01 s se recupera el 95 % en unos 30 ms y el 99.3 % en 50 ms según esta ecuación. La prueba de liberación comprueba esa recuperación matemática; no simula el tiempo mecánico de rearranque de una rueda bloqueada.

## Pérdida de dirección al bloquear

El círculo ya reduce Fy al consumir capacidad longitudinal, pero no impone necesariamente Fy=0. Se extendió el modo especial con factor lateral cero y longitudinal unitario sobre el límite ya reducido. También se anuló la rotación de la fuerza longitudinal con la dirección dentro de ese modo. Así se evita generar una fuerza lateral residual mediante Fx*sin(delta). El resultado es una aproximación deliberada, no un modelo completo de la fricción durante un deslizamiento arbitrario.

## Una interfaz reproducible

Las aceleraciones anteriores y las memorias de adherencia/bloqueo inicialmente entraban como argumentos adicionales. Pasarlas a z dejó cuatro entradas `z,u,p,dt` y dos salidas `z_next,tele`. La comparación guardada verificó que la reorganización no cambiaba las trayectorias de cinco familias de ensayos. Los dos componentes de u siguen siendo fuerza total y dirección; todavía no hay vector de actuación independiente por rueda.

## Del ensayo aislado a un circuito

La integración con Pure Pursuit de la práctica 4 permitió someter la planta a curvas interiores, exteriores y rectas sin reescribir su física. Se guardaron copias independientes para no alterar los originales. Las animaciones usan posiciones y guiñada simuladas, y la flecha de velocidad transforma vx/vy al marco global.

## Planificar antes de controlar velocidad

Una velocidad constante era inadecuada para comparar rectas con curvas. V9 construyó un perfil por curvatura y propagó límites de aceleración/frenado alrededor de un circuito cerrado. El PI recibe ese perfil. El techo de velocidad y la velocidad alcanzada son distintos: varios techos altos de V9 producen prácticamente la misma vuelta porque domina la curvatura.

## Crear una demanda que exponga los límites

V10 amplió el circuito de 8 a 14 curvas, introdujo chicane y curvas cerradas de 90° y 70°, y colocó una recta de 172 m antes de la primera frenada fuerte. El conductor pasa a objetivos de fuerza máxima con aplicación exponencial rápida; no se añadió un escalón ideal. La saturación y el bloqueo aproximado se conservan en los resultados, sin ocultarlos mediante un controlador de estabilidad.

## Frenar más tarde sin alterar la planta

V11 elevó ayMax de 8 a 11 y axFreno de 8 a 12 m/s², y retiró 0.10 s de anticipo adicional. “Frenar menos” se concretó en reducir menos la velocidad al llegar a las curvas, no en bajar la fuerza máxima del freno. A 35 m/s el mapeo retrasa el inicio planificado en 25.5 m en la primera recta. La vuelta mejora de 51.384 a 47.680 s, mientras el tiempo de bloqueo aproximado aumenta de 12.276 a 19.869 %. Un plan más optimista puede funcionar en este circuito sin ser universalmente realizable.

## Separar visualización 3D de física 3D

Se eligieron herramientas gráficas nativas, sin añadir una biblioteca ni un vehículo mecánico distinto. La carrocería tiene volumen visual y la cámara sigue la guiñada, pero no se inventan balanceo, cabeceo ni giro de rueda. Los MP4 son renderizados de las historias, con duración dependiente de su fps; el último fotograma puede añadir una fracción de segundo.

El proceso de vídeo fue: calcular/guardar la historia, desactivar el temporizador interactivo, actualizar la escena en instantes regulares, obtener fotogramas de la figura con `getframe`, codificar con `VideoWriter` en MPEG-4 y verificar lectura/duración con `VideoReader`. Se cierra la figura y se elimina el temporizador al terminar. Es un renderizado de la figura propia, no una captura continua del escritorio. Para acelerar exportaciones, los conos pueden agruparse en un único objeto gráfico por color manteniendo su geometría; esto no cambia los estados simulados. V11 3D se guardó a 30 fps y V12 a 15 fps, con reproducción a tiempo real en ambos casos.

## Aceleración extrema V12

La demanda de tracción subió de 2500 a 5000 N y tauAcel bajó de 0.030 a 0.010 s. Frenada, mapeo, circuito y ecuaciones de la planta se mantuvieron. La vuelta empeoró a 149.036 s y el error máximo creció a 4.347 m. La demanda excede el límite longitudinal trasero durante el 98.556 % de las muestras acelerando, frente al 49.664 % de su base. Es una condición de estrés útil para la coordinación futura, no una mejora de prestaciones.

## Decisiones pendientes

La siguiente fase mantiene congelada la planta y monta la arquitectura por bloques. Deben definirse referencia de guiñada, señales de estimación, asignación por rueda, saturaciones de actuadores y coordinación. Para ABS/TCS basados en slip físico faltan velocidad angular, radio e inercia de rueda y una ley fuerza–deslizamiento. La convención de deriva, el modo de bloqueo y los parámetros de adherencia requieren revisión/calibración antes de conclusiones sobre un vehículo real.
