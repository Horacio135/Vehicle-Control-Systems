# Intermediate presentation — English reading copy

[← Documentation](README.md) · [Original presentation](SCV_Evaluaci%C3%B3n_Intermedia_Proyecto_HHRB.pptx) · [Current project status](../README.md)

English reading copy of the original thirteen-slide milestone. Its development statuses refer to that milestone, not to the final V12 snapshot. Original figures and slide layouts remain available in the archived presentation.

## 1 · Stability control for an autonomous vehicle

**Improvement proposal: Vehicle Control Systems.** A four-wheel dynamic model, ESC and ABS/TCS coordination, aiming for better control and performance during cornering.

## 2 · What problem are we addressing?

Evasive manoeuvres and high-speed cornering can undermine yaw control near the adhesion limit.

- **Understeer:** the car turns less than requested and opens its trajectory.
- **Oversteer:** excessive rotation makes path tracking difficult.
- **Consequences:** slower cornering, reduced safety and a risk of leaving the road.

The objective is to improve stability without unnecessary braking or exceeding the tyre's physical limits.

## 3 · Understeer and oversteer

The original illustrations compare the two behaviours. Both can result in loss of vehicle control.

## 4 · Why choose ESC?

Selective, short braking interventions can generate an additional yaw moment and rotate the vehicle towards its intended response.

| Interface | Proposed information |
|---|---|
| ESC inputs | Steering angle, longitudinal speed and actual yaw rate |
| ESC action | Additional yaw moment addressing the difference between desired and actual yaw response |

The proposed advantage is to improve cornering stability while minimizing changes to vehicle speed. This is the intended control strategy, not a demonstrated result of the current implementation.

## 5 · ESC alone is insufficient near the limit

Differential braking can lose effectiveness if lock and traction loss are unmanaged.

- **ESC without ABS:** excessive braking can lock a wheel and reduce lateral-force and yaw-control authority.
- **ESC without TCS:** excess motor torque can induce traction loss and consume adhesion needed for steering.
- **ESC + ABS + TCS:** coordination aims to regulate slip and preserve useful forces for stability control.

An isolated ESC may therefore provide limited benefits; ABS/TCS coordination is intended to produce a more robust solution.

## 6 · Development roadmap at the intermediate milestone

| Phase | Scope | Status in the original presentation |
|---|---|---|
| 1 · Four-wheel model | Structure and individual contact forces | Developed |
| 2 · Tyres and loads | Combined adhesion and load transfer | Under validation |
| 3 · ABS + TCS | Wheel rotational dynamics and slip | Pending |
| 4 · Basic ESC | Yaw control and differential braking | Pending |
| 5 · Integration | Coordination, testing and tuning | Pending |

## 7 · Improvements to demonstrate

The benefits must be measured against a baseline without ESC:

- Reduce yaw deviations and excessive transient responses.
- Reduce path-tracking error in demanding manoeuvres.
- Brake and drive without wasting available adhesion.

Planned evaluation measures include yaw error, path error, adhesion utilization and stability during manoeuvres. These are project hypotheses rather than completed ESC validation.

## 8 · Four-wheel physics and combined adhesion

The chassis retains three planar degrees of freedom; contact calculations are performed wheel by wheel.

- Four slip angles and individual tyre forces.
- Rear-wheel drive and 60/40 front/rear brake allocation.
- A shared Fx/Fy friction circle.
- Per-wheel telemetry.

The 5 m/s skidpad trajectory opens slightly relative to the kinematic path because of tyre slip angles.

## 9 · Skidpad behaviour at 5 m/s

The original plots compare kinematic and dynamic yaw rates and friction-circle utilization. In a mild turn, adhesion is not saturated and the dynamic yaw rate remains close to the kinematic response.

## 10 · Load transfer and variable lateral stiffness

Acceleration changes vertical load allocation and therefore each tyre's response. The model includes longitudinal/lateral load transfer, individual Fz with total mg, and lateral stiffness dependent on Fz.

The 5 m/s slalom shows periodic reversal of wheel loads and illustrates the implemented load-transfer effect.

## 11 · Steering changes in a 5 m/s slalom

The figures show load transfer, per-wheel lateral-stiffness variation and adhesion utilization during alternating steering. In that recorded test, stiffness changes with Fz and adhesion utilization remains below one.

## 12 · Adhesion saturation at 14 m/s

The 14 m/s slalom is a limiting scenario in which wheel forces reach their adhesion bounds. The recorded results show saturation across all wheels.

## 13 · Additional views of the saturation scenario

Further plots from the same 14 m/s slalom illustrate the adhesion limits. They form part of the intermediate evidence, alongside the current repository's later tracking, speed-planning and aggressive-driver campaigns.
