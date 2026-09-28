# PerformanceSim

A C++ vehicle performance simulator. Reads vehicle specifications from text files, simulates straight-line acceleration using a time-stepped physics model, and exports per-timestep telemetry to CSV for plotting.

Code-only individual course project (5 mini-projects, each building on the last).

## Features

- Time-stepped longitudinal dynamics (aerodynamic drag, rolling resistance, tire grip limit)
- Engine torque as a function of RPM (stepped torque curve scaled from peak torque and redline)
- 5-speed gearbox with automatic upshifts at redline and a fixed shift-duration power interruption
- Milestone detection with linear interpolation inside the timestep: 0-60 mph, quarter-mile time
- Top speed detection, reported as gear limited (redline in top gear) or converged (acceleration ~0 in top gear)
- Vehicle data loaded from external text files
- Telemetry export per vehicle: time, speed, acceleration, gear, RPM, distance

## Physics Model

Each timestep (`dt = 0.01 s`):

```
F_drag     = 0.5 * rho * Cd * A * v^2
F_roll     = Crr * m * g
F_traction = min( T(rpm) * gearRatio * finalDrive * efficiency / r,  mu * m * g )
F_net      = F_traction - F_drag - F_roll
a          = F_net / m
```

Engine speed from vehicle speed:

```
RPM = (v / r) * gearRatio * finalDrive * 60 / (2 * pi)
```

During a shift (`0.20 s`), traction force is zero; drag and rolling resistance continue to act, so the car briefly decelerates.

### Constants

| Constant | Value |
|---|---|
| Air density | 1.225 kg/m^3 |
| Gravity | 9.81 m/s^2 |
| Rolling resistance coefficient | 0.015 |
| Drivetrain efficiency | 0.90 |
| Gear ratios | 3.50, 2.10, 1.40, 1.00, 0.65 |
| Final drive | 3.73 |
| Idle RPM | 1000 |
| Shift duration | 0.20 s |
| Max simulated time | 180 s |

### Torque curve (fraction of peak torque, by fraction of redline)

| RPM / redline | Torque / peak |
|---|---|
| < 0.15 | 0.50 |
| 0.15 - 0.30 | 0.75 |
| 0.30 - 0.45 | 0.95 |
| 0.45 - 0.65 | 1.00 |
| 0.65 - 0.85 | 0.90 |
| 0.85 - 1.00 | 0.75 |
| > 1.00 | 0.00 |

## Project Structure

```
PerformanceSim/
  Performance Sim.cpp
  input/      vehicle spec files (.txt)
  results/    generated telemetry CSVs
```

Both `input/` and `results/` must exist before running. The program does not create directories.

## Input File Format

Plain text. Line 1 is the vehicle name; line 2 onward holds seven whitespace-separated numbers in this order:

```
mass (kg)
drag coefficient
frontal area (m^2)
wheel radius (m)
tire friction coefficient
peak torque (N*m)
redline (RPM)
```

Example `input/m3.txt`:

```
M3 Competition
1730
0.35
2.28
0.34
1.0
650
7200
```

Files with missing fields or non-positive values are rejected and skipped.

## Build and Run

Visual Studio: create a C++ Console App, add the source file, build and run (F5). Set the working directory so `input/` and `results/` resolve.

Command line (g++):

```
g++ -std=c++17 -o performancesim "Performance Sim.cpp"
./performancesim
```

Vehicle files to load are listed in `main()`. Edit that list to add or remove vehicles.

## Output

Console, per vehicle:

```
Loaded vehicle: M3 Competition
Gear shifted at 2.16 seconds from 1 to 2 at 44.08 mph
0-60 mph in 3.300 seconds
Quarter mile time: 11.400 seconds
Top speed reached at: ... mph at ... seconds
Data exported to results/M3 Competition_telemetry.csv
```

CSV columns (`results/<vehicle name>_telemetry.csv`):

```
Time_s, Speed_mph, Acceleration_G, Gear, Engine_RPM, Distance_m
```

Open in Excel or Google Sheets to plot speed vs time, acceleration vs time, and distance vs time. Overlay two vehicles' files on one chart for direct comparison.

## Validation

Results were compared against published 0-60, quarter-mile, and top-speed figures for a test set of production vehicles (sedans, muscle cars, sports cars, trucks). Acceleration-phase numbers land close to published figures. Top speed reflects the physical drag/traction equilibrium, not manufacturer electronic limiters, so simulated top speeds can exceed published limited values.

## Known Limitations

- One shared gearbox (ratios, final drive) for every vehicle
- Stepped torque curve rather than a continuous or interpolated curve
- No electronic speed limiter
- Single-speed EV drivetrains are not modeled (multi-gear logic is applied to all vehicles)
- Fixed drivetrain efficiency for all vehicles
- No braking or deceleration phase; straight-line acceleration only
