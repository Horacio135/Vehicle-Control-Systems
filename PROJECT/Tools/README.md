# Reproduction and verification tools

[← Project](../README.md) · [Run sources](../Results/Run_Sources/README.md)

## Reproduce a simulation

`reproduce_run.m` executes an archived source snapshot and saves full-resolution results. It requires MATLAB, not Python or an existing video.

From the repository root:

```matlab
addpath('PROJECT/Tools');
outputFile = reproduce_run('V12_35_3D.m', fullfile(tempdir,'vehicle_v12'));
```

Autoplay is disabled in a temporary copy only; equations and scenario parameters are unchanged. Original Live Scripts and snapshots remain intact. Scenario figures are created without display, and a complete MAT file is saved. No MP4 is automatically rerendered. The temporary copy remains available for traceability.

## Verify the published archive

```bash
python PROJECT/Tools/verify_publication.py
```

The script and `publication_checks.json` verify local Markdown links, archived hashes, twelve stage pages, source exports and result-data coverage. These are publication checks, not experimental physical validation.

The [MATLAB reproduction record](matlab_reproduction_check.json) confirms that the organized V11 35 m/s snapshot reproduced a 47.680 s lap and 0.775982 m maximum error. Finite states, unchanged sources and timer cleanup were checked. The entire historical collection has not been rerun.

## English-localization checks

The [source-integrity report](localization_source_integrity.json) checks 58 MATLAB source representations against the pre-localization Git revision. Executable non-text tokens are unchanged, and all 22 MLXs retain their cached outputs and media. Comments, explanatory prose and display messages are translated; historical identifiers are retained.

The [MATLAB regression report](english_localization_checks.json) records successful native conversion of all 22 MLXs and exact numerical equality for both V11 and V12 after translation. States, forces, steering, speed references, lateral error, adhesion factors and lock flags match the previous results; timers are cleaned up.
