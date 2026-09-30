# PerformanceSim

A C++ vehicle performance simulator. Reads vehicle data from text files, simulates straight-line acceleration using a time-stepped physics model, and exports per-timestep telemetry to CSV for plotting.

## Features

- Time-stepped longitudinal dynamics
- Engine torque as a function of RPM
- 5-speed gearbox with automatic upshifts at redline and a fixed shift-duration power interruption
- Milestone detection with linear interpolation inside the timestep: 0-60 mph, quarter-mile time
- Top speed detection
- Vehicle data loaded from external text files
- Telemetry export per vehicle: time, speed, acceleration, gear, RPM, distance
  
## Known Limitations

- One shared gearbox for every vehicle
- Stepped torque curve rather than a continuous or interpolated curve
- No electronic speed limiter
- EV drivetrains are not modeled
- Fixed drivetrain efficiency for all vehicles
- No braking or deceleration phase; straight-line acceleration only
## Validation

| Vehicle | Sim 0-60 | Published 0-60 | Error | Sim 1/4 mi | Published 1/4 mi | Error |
|---|---|---|---|---|---|---|
| Corolla (2024 LE CVT) | 7.505s | 7.1s | +5.7% | 16.103s | 15.5s | +3.9% |
| Mustang GT (2024, manual) | 3.680s | 4.3s | -14.4% | 12.072s | 12.9s | -6.4% |
| Challenger Hellcat | 3.263s | 3.6s | -9.4% | 11.583s | 11.0s | +5.3% |
| Civic Si (2023) | 6.011s | 6.5s (approx.) | -7.5% | 14.758s | 14.9s (approx.) | -0.9% |
| BMW M3 (2023, manual) | 3.899s | 4.1s | -4.9% | 12.412s | 12.3s | +0.9% |
| Porsche 911 GT3 (992, manual) | 3.339s | 3.8s (approx.) | -12.1% | 11.437s | 11.4s (approx.) | +0.3% |
| Camaro SS (2016+) | 3.430s | 4.0s | -14.3% | 11.867s | 12.3s | -3.5% |
| Silverado (5.3L V8) | 5.870s | 6.5s (approx.) | -9.7% | 14.641s | 15.0s (approx.) | -2.4% |
| **Average absolute error** | | | **~9.7%** | | | **~3.0%** |

## Data Sources

| Vehicle | Specs used | Source |
|---|---|---|
| Corolla (2024 LE CVT) | Curb weight, torque, redline, published 0-60/quarter mile | motormatchup.com |
| Mustang GT (2024, manual) | Curb weight, torque, redline | thedrive.com, carpro.com |
| Mustang GT (2024, manual) | Published 0-60/quarter mile | thedrive.com |
| Challenger Hellcat | Curb weight, torque | dupontregistry.com, tflcar.com |
| Challenger Hellcat | Published 0-60/quarter mile | dupontregistry.com, carscoops.com |
| Civic Si (2023) | Curb weight, torque, redline | hondanews.com |
| Civic Si (2023) | Published 0-60/quarter mile | approximate|
| BMW M3 (2023 Sedan, manual) | Curb weight, torque, redline, published 0-60/quarter mile | motormatchup.com |
| Porsche 911 GT3 (992, manual) | Curb weight, torque, redline | supercars.net |
| Porsche 911 GT3 (992, manual) | Published 0-60 | fastcar.co.uk |
| Porsche 911 GT3 (992, manual) | Published quarter mile | approximate |
| Camaro SS (2016+) | Curb weight, torque, published 0-60/quarter mile | lsxmag.com |
| Silverado (5.3L V8) | Curb weight, torque, published 0-60/quarter mile | approximate|

drag coefficient (Cd), frontal area, and tire friction coefficient (μ) were estimates based on values for each vehicle class, not actual figures.
