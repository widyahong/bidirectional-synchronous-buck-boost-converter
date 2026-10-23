# 4-Switch Bidirectional Synchronous Buck-Boost Converter

Open-loop power stage between **PORT_A** and **PORT_B**. Each port can be a source, a load, or a bidirectional supply.
**No sensors, no voltage/current regulation.** Everything is set by variables in code or Serial.
A and B switch **together** with **one duty D**; the low sides are the ones that are OFF in the opposite phase.

> This is **not a regulated converter**: there is no CC/CV, no cut-off, no protection. Use supplies/loads you can
> babysit, current-limited at first, and fuses on both sides.

## 1. Generic MCU requirements

| Need | Detail |
|---|---|
| 4 digital outputs | `HIN_A`, `LIN_A`, `HIN_B`, `LIN_B` (3.3 V logic, active-high) |
| PWM 20-200 kHz | 10+ bit resolution preferred, **all 4 outputs from the same timer** |
| Offset per output | Used to make dead-time (hardware dead-time generator also works) |
| Serial | Any UART/USB-CDC |

The rest of this document is the **ESP32 example** (Arduino IDE, no extra libraries).

## 2. Operating principle

Two diagonal switch pairs alternate, separated by dead-time:

| Phase | Q1 (HS_A) | Q2 (LS_A) | Q3 (HS_B) | Q4 (LS_B) | Inductor |
|---|---|---|---|---|---|
| D (charge) | ON | off | off | ON | charged from PORT_A |
| dead-time | off | off | off | off | freewheel via body diodes |
| 1-D (release) | off | ON | ON | off | released into PORT_B |
| dead-time | off | off | off | off | freewheel via body diodes |

Zero-current point (**balance duty**):

```
D0 = VB / (VA + VB)        (VA = PORT_A voltage, VB = PORT_B voltage; equal voltages -> D0 = 50 %)
```

- `D > D0` -> energy flows **A -> B** (PORT_A supplies, PORT_B receives)
- `D < D0` -> energy flows **B -> A** (PORT_B supplies, PORT_A receives)

So the firmware exposes **direction** and **offset** instead of a raw duty:

```
D = D0 + offset      (direction A2B)
D = D0 - offset      (direction B2A)
```

`offset` (duty points away from balance) indirectly sets current strength, unregulated:

```
I_avg ~ (VA + VB) * offset / R_loop
```

`R_loop` (Rds(on) + inductor DCR + wiring + source/load internal resistance) is small, so **offsets of tenths of a percent
can already mean amps**. Duty resolution is also coarse (1 counter step is about 0.1 % at 50 kHz / 10 bit), so
the current changes in big steps. Lower `freq` gives more bits and finer steps.

`VA` and `VB` are only **numbers you type in** (nominal volts) to compute `D0`. Real port voltages drift with
load and source behavior, so re-measure with a multimeter and restart when they change.

## 3. Circuit (power stage)

The gate driver is a black box (your discrete driver: 2 half-bridges, 4 logic inputs, bootstrap high-side, no EN pin).

```
 PORT_A+ --[F1 10A]--+-----------+                        +-----------+--[F2 10A]-- PORT_B+
                     |           |                        |           |
                   C1||C2     Q1 (HS_A)                Q3 (HS_B)    C3||C4
                     |           |                        |           |
                     |           +-- SW_A --[ L1 47uH ]-- SW_B --+    |
                     |           |                        |           |
                     |        Q2 (LS_A)                Q4 (LS_B)      |
                     |           |                        |           |
 GND ----------------+-----------+------------------------+-----------+----- GND (common)

 Q1..Q4 gate/source <---> [ DISCRETE MOSFET DRIVER ]

                  +-----------------------+
 GPIO25 --+------>| HIN_A      (Q1)       |
 GPIO26 --+------>| LIN_A      (Q2)       |
 GPIO32 --+------>| HIN_B      (Q3)       |
 GPIO33 --+------>| LIN_B      (Q4)       |
 ESP32 GND ------>| GND (common with      |
                  | power GND)            |
                  +-----------------------+
   R1..R4 = 10k pull-down from each GPIO line to GND (gates stay OFF during boot/reset)
```

### Bill of materials (all through-hole)

| Ref | Part | Note |
|---|---|---|
| Q1-Q4 | IRFZ44N (TO-220) | 55 V; not logic-level, driver required |
| L1 | 47 uH toroidal power inductor | Isat >= 10 A, wire for >= 5 A continuous |
| C1, C3 | 470 uF / 35 V electrolytic, radial | one per port |
| C2, C4 | 100 nF / 50 V film, radial | close to the MOSFETs |
| F1, F2 | 10 A fuse, 5x20 mm with PCB holder | one per port |
| R1-R4 | 10 kOhm, 1/4 W axial | pull-downs on the 4 logic lines |
| MCU | ESP32 DevKit (ESP32-WROOM-32) | 3.3 V logic |
| Driver | your discrete MOSFET driver | bootstrap high-side |

Target: 12-24 V per port, about 5 A, 50 kHz.

## 4. ESP32 implementation

Uses the ESP-IDF LEDC driver (`driver/ledc.h`, included with the Arduino-ESP32 core, no library to install).
It is used directly (not through `ledcAttach`) so that **all four channels share one timer**; that keeps
Q1/Q4 and Q2/Q3 in lockstep, which this mode requires.

| Signal | GPIO | LEDC channel | Waveform (P = period counts, dt = dead-time counts) |
|---|---|---|---|
| HIN_A (Q1) | 25 | 0 | high in `[0, D*P)` |
| LIN_B (Q4) | 33 | 3 | high in `[0, D*P)` |
| LIN_A (Q2) | 26 | 1 | high in `[D*P+dt, P-dt)` |
| HIN_B (Q3) | 32 | 2 | high in `[D*P+dt, P-dt)` |

- Dead-time is built with the LEDC `hpoint` offset, so HS/LS of one leg never overlap.
- Duty is limited to **5-90 %** (bootstrap needs low-side time). `D0` must also be inside this range.
- On `start`: both low sides ON for 5 ms (bootstrap charge, inductor shorted to GND, no current), then PWM begins
  **at D0 (zero current)** and ramps toward `D0 +/- offset`.
- Ramp step is clamped below the dead-time, so a live update cannot cause shoot-through.
- `stop` forces all four outputs LOW (all MOSFETs off).

## 5. Variables (top of `firmware.ino`) and Serial commands (115200 baud)

| Command | Variable | Default | Meaning |
|---|---|---|---|
| `va <V>` / `vb <V>` | `vA` / `vB` | 12.0 | nominal port volts, only used to compute D0 (stopped only) |
| `dir a2b` / `dir b2a` | `direction` | A2B | energy flow direction (live) |
| `off <%>` | `offsetPct` | 0.0 | duty points away from D0, 0-5 (live, ramped) |
| `freq <Hz>` | `freqHz` | 50000 | 20000-200000 (stopped only) |
| `dead <ns>` | `deadNs` | 300 | dead-time (stopped only); tune with a scope |
| `step <%>` / `rate <ms>` | `rampStep` / `rampMs` | 0.1 / 50 | ramp speed (live) |
| `start` / `stop` | - | stopped | - |
| `status` / `help` | - | - | - |

## 6. Bring-up checklist

1. Flash with the power stage unpowered; scope the 4 GPIOs (driver disconnected): Q1/Q4 identical, Q2/Q3 identical,
   dead-time visible between Q1 and Q2 and between Q3 and Q4.
2. Connect the driver. Power with current-limited bench supplies (e.g. 12 V, 0.5 A limit) at both ends or a resistor load.
3. `start` with `off 0`, check SW_A / SW_B, then raise `off` in very small steps.
4. Only then connect the real sources/loads, with correct `va`/`vb`.
