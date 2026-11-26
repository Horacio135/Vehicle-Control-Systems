# Documentation

[Latest decision: Q restoration and retained moose test](Design_Decisions/Q_Restoration_and_Moose_Test_2026-11-26.md).

[← Project](../README.md) · [Versions](Versions/README.md) · [Design decisions](Design_Decisions/README.md) · [Results](../Results/README.md)

**Design rationale, theory and evidence from V1 through the complete estimated loop.** The current milestone is dated **26 November 2026**; the original V1–V12 archival snapshot remains dated 22 November. Figures and runs retain their own model/configuration rather than being relabelled as current evidence.

## Reading paths

| If you want to… | Read… |
|---|---|
| Run the current complete loop | [Current State](../Current%20State/README.md) |
| Read the physical derivation and rejected alternatives | [Dynamics development](Dynamics_Development.md) |
| Understand the Q adjustment and straight-line departure | [Estimator and failure analysis](Estimator_Q_and_Failure_Analysis.md) |
| Review recent design decisions and rejected alternatives | [Post-V12 decision ledger](Design_Decisions/Post_V12_Decisions.md) |
| Follow V1–V12 | [Version history](Versions/README.md) |
| Continue beyond V12 | [Later milestones](Versions/After_V12/README.md) |
| Understand the earlier decisions | [Design decisions](Design_Decisions/README.md) |
| Check numerical conclusions | [Results](../Results/README.md) |
| Trace original V1–V12 artifacts | [Original manifest](artifact_manifest.json) |
| Trace current source and new evidence | [Closed-loop manifest](closed_loop_artifact_manifest.json) |
| Understand historical result provenance | [Numerical evidence](../Results/Numerical/README.md) |
| Review documentation presentation choices | [README design references](README_References.md) |

## Intermediate presentation

The [English presentation transcript](Intermediate_Presentation_English.md) covers all thirteen slides. The [original presentation](SCV_Evaluaci%C3%B3n_Intermedia_Proyecto_HHRB.pptx) retains the intermediate milestone: understeer/oversteer, ESC choice, future ABS/TCS coordination, tyre forces and transfer of load. Its pending-work list is historical, not a description of the current package.

The current loop includes force-based ABS/TC and seven-state estimation; an ESC yaw law and physical-slip regulators remain pending. Neither the old slides nor the current successful lap establish experimental validity.

## Archive and language policy

Earlier V1–V12 publication localized explanatory source text into English without changing executable equations; that history is preserved. **Current MATLAB identifiers and comments remain Spanish as requested, while the surrounding documentation is English.** Original filenames and recorded graphic/video labels are retained where needed for provenance.

The manifests record source and published hashes. Identical media are stored once and mapped in the newer manifest. Autosaves, backups, credentials and machine configuration are excluded. Source/metadata localization must never be presented as a new physical simulation.

**Source Exports** provides readable MLX code without creating same-name script shadowing. **Run_Sources** preserves older configurations. V7/V9 compatibility limits remain documented rather than silently fixed. The [current package](../Current%20State/README.md) supplies the active hybrid plant, sensors, EKF and external integration adapters.

The original pre-localization V1–V12 source remains available in [the archived Git revision](https://github.com/Horacio135/Vehicle-Control-Systems/tree/b52aeef7d87d04282984e22a5ac1cce60275f92e).
