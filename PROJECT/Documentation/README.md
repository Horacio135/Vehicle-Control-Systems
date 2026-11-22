# Documentation

[← Project](../README.md) · [Versions](Versions/README.md) · [Design decisions](Design_Decisions/README.md) · [Results](../Results/README.md)

**Design rationale, development stages and evidence through V12.** This record is reconstructed from code, saved artifacts and the working-session decisions. It does not invent intermediate dates, measurements or commits. The archival snapshot is dated **22 November 2026**; figures and runs retain their historical configurations.

## Reading paths

| If you want to… | Read… |
|---|---|
| Follow the evolution from V1 to V12 | [Version history](Versions/README.md) |
| Understand why a modelling choice was made | [Design decisions](Design_Decisions/README.md) |
| Connect a controller to the plant | [Plant contract](../Dynamic%20Model/README.md#plant-interface) |
| Check numerical conclusions | [Results](../Results/README.md) |
| Trace an archived artifact | [Artifact manifest and hashes](artifact_manifest.json) |
| Understand result provenance | [Numerical evidence](../Results/Numerical/README.md) |
| Review documentation presentation choices | [README design references](README_References.md) |

## Intermediate presentation

Read the [English presentation transcript](Intermediate_Presentation_English.md) for all thirteen slides. The [original intermediate presentation](SCV_Evaluaci%C3%B3n_Intermedia_Proyecto_HHRB.pptx) is retained as the recorded milestone. It describes understeer/oversteer, the ESC choice, future ABS/TCS coordination, tyre forces and load transfer. It represents an **intermediate milestone**, not the final V12 state.

ABS/TCS, basic ESC and integration are listed as pending. Improvements in stability and performance are hypotheses to be measured, not demonstrated benefits of an implemented ESC. Recent videos and tables extend the evidence but still do not establish experimental validity.

## Archive policy

Files retain their historical names. Source comments, instructions and display messages have been localized into English; identifiers, parameters and executable equations are unchanged. The manifest records original-source hashes as well as current localized hashes. Recorded figures, videos and the intermediate presentation retain their original bytes. Exclusions are `.asv` autosaves, `.bak` backups, `Respaldo_*` directories, an `untitled.m` scratch file and a duplicate skidpad helper. Credentials, machine configuration and absolute user paths are not published in the manifests.

**Source Exports** provides readable representations of MLX code. **Run_Sources** preserves the configurations that generated historical results, which can differ from current files. V7/V9 compatibility limitations are documented rather than silently fixed. The plant remains frozen for the next block-architecture phase.

The documentation, MATLAB explanatory text and newly generated plot labels are in English. Previously recorded plots, videos and presentation slides remain historical artifacts; their English explanations and measurement definitions are provided alongside them. The original pre-localization source is available in [the archived Git revision](https://github.com/Horacio135/Vehicle-Control-Systems/tree/b52aeef7d87d04282984e22a5ac1cce60275f92e).
