# Retained moose-style test 02 scenario

[← Current package](../README.md) · [Saved result and limitations](../../Results/Moose_Test_02/README.md)

```matlab
cd(fullfile('PROJECT','Current State','Ensayos_alce_seguimiento'))
open('Ejecutar_alce_seguimiento.mlx')
% Run the entire Live Script to simulate and render only Alce_PP_02.
```

The `.m` entry point is equivalent. MATLAB R2026a was used. The scenario calls the current Pure Pursuit, longitudinal, ABS/TC and dynamic functions without changing their interiors. Sensors, estimator and ESC are bypassed. Only test 02 remains; rerunning does not recreate discarded campaigns.

Outputs go to `PROJECT/Results/Moose_Test_02` and overwrite only this retained scenario's evidence. The published copy replaces workstation-specific paths with paths derived from the script location. Recorded data are unchanged. `validar_alce_seguimiento` resolves historical plant/control source paths locally; it excludes `configuracionIntegracion.m` from historical hash comparison because Q was restored after recording and is not used by this manoeuvre.

The fixed target is 20 m/s; actual speed is governed by forces, tyre limits and TC. It is not clamped. See the results page for the failure, no-braking policy and termination condition.
