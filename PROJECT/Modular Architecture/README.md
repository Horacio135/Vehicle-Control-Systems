# Modular architecture · Stage 1

[← Project](../README.md) · [Frozen plant specification](../Dynamic%20Model/README.md#plant-interface)

**This template currently runs the frozen dynamic plant only.** The original Live Scripts are retained. Five separate sensing, estimation, lateral, longitudinal and emergency functions are now available in [Blocks](../Blocks/README.md), alongside the relocated plant and initializer. They are not yet connected in this simulation loop; TC, ABS and ESC are not implemented.

## Files

| File | Role |
|---|---|
| [pasoDinamico.m](../Blocks/pasoDinamico.m) | Standalone dynamic plant, relocated without changing its equations |
| [inicializarEstadoDinamico.m](../Blocks/inicializarEstadoDinamico.m) | Existing initializer for six motion variables or the complete state |
| [runSimulation.m](runSimulation.m) | Editable parameters, initial conditions, scenario inputs, integration loop and basic plots |
| [plant_provenance.json](../Blocks/plant_provenance.json) | Extraction source, hashes and input/output contract |
| [extraction_checks.json](../Blocks/extraction_checks.json) | MATLAB checks against the unchanged current local V8 |

## Run

From the repository root in MATLAB:

```matlab
run(fullfile('PROJECT','Modular Architecture','runSimulation.m'));
```

The script adds the sibling Blocks directory to the path and checks that another function with the same name does not shadow this plant. If that check fails, set Current Folder to Blocks before running.

Verified default result: 7.472 s including acceleration and braking, peak vx 35.002 m/s and total travelled distance 140.014 m. All next states and telemetry fields exactly match the original over 3736 steps. Another 700 steps exercise steering, traction loss, lock mode and full-release recovery with the same exact match.

Edit `cfg` and `p` near the top of `runSimulation.m`. The default test starts from rest, applies full driving-force demand to reach 35 m/s, and brakes to the stopping threshold. The target-speed event is part of the scenario, not an implemented speed, traction or ABS controller. Plots are generated, but no animation or video starts automatically.

## Dynamic-block call

```matlab
z = inicializarEstadoDinamico([X;Y;psi;vx;vy;r]);
u = [requestedTotalForce; frontWheelSteeringAngle];
[z_next, tele] = pasoDinamico(z, u, p, dt);
z = z_next;
```

Inputs are the complete current state, requested input, vehicle parameters and time step. Outputs are the complete next state and step telemetry. Force is in N and steering is in rad.

The complete state has twenty components: six motion variables, two previous accelerations, four lateral-capacity memories, four braking-capacity memories and four approximate lock-mode flags. Always propagate all twenty; do not rebuild only the first six on each iteration.

## Timing and result access

```matlab
history.z(k,:)          % State at t_k
history.u(k,:)          % Input applied during step k
history.tele{k}         % Telemetry calculated for that step
history.z(k+1,:)        % Next state at t_k + dt
history.tele{k}.Fxw     % Per-wheel longitudinal forces
history.tele{k}.Fyw     % Per-wheel lateral forces
history.tele{k}.Fzw     % Per-wheel vertical loads
```

`history.t` indexes states. `history.tInput` indexes inputs and telemetry. States therefore have one more row than the latter. The harness stops instead of altering the physical equations if it encounters non-real/non-finite states or negative wheel loads.

## Connection to the requested architecture

The intended later chain is noisy sensor measurements → state estimator → combined longitudinal/lateral controller → emergency arbitration → coordinated TC/ABS → dynamic model, with physical outputs fed back to the sensor block. The current file marks the insertion point but does not add identity functions that could be mistaken for implemented blocks.

For this plant-only test, the true state is used only to schedule the scenario. Once sensing/estimation is connected, controllers should receive the estimated state rather than bypass that chain.

**Interface boundary still to resolve:** the requested diagram includes independent wheel-force commands, whereas the frozen V8 plant accepts one total force and uses its fixed drive/brake allocation. This extraction retains that interface. It does not accept four independent force commands or claim selective braking by summing them. That future interface decision is separate from this unchanged physical block.
