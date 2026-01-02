Modules:
-----------------------
- [ ] Enter modules in the order specified in ModuleUtils.ModulePosition

ModuleConstants: (real and simulation)
-----------------------
- [ ] Wheel Diameter
- [X] Coupling Ratio
- [ ] Velocity At 12 Volts
- [X] Modules locations (in meters)

Encoder:
----------------------
- [ ] Encoder ID
- [ ] Sensor Range (should be PlusMinusHalf)
- [ ] Sensor Direction (should be CounterClockwise in default sds module)

Steer:
-----------------------
- [ ] Motor ID
- [ ] Inverted
- [X] Neutral Mode
- [X] Current Limit
- [X] Gear Ratio (should use RotorToSensorRatio)
- [X] Encoder Usage and ID (should use fuse)
- [ ] FF (ks, kv, ka)
- [ ] PID
- [X] Use ContinuousWrap
- [X] Control mode (motion magic, voltage, torque)
- [X] Enable/Disable FOC (only talonfx)

SteerSimulation:
-----------------------
- [X] Moment of inertia
- [X] DCMotor

Drive:
-----------------------
- [ ] Motor ID
- [ ] Inverted
- [X] Neutral Mode
- [X] Current Limit
- [X] Gear Ratio (should use RotorToSensorRatio)
- [ ] FF (ks, kv, ka)
- [ ] PID
- [X] Control mode (motion magic, voltage, torque)
- [X] Enable/Disable FOC (only talonfx)

DriveSimulation:
-----------------------
 - [X] Moment of inertia
 - [X] DCMotor
