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
