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
- [x] Motor ID
- [x] Inverted
- [X] Neutral Mode
- [X] Current Limit
- [X] Gear Ratio (should use RotorToSensorRatio)
- [X] Encoder Usage and ID (should use fuse)
- [x] FF (ks, kv, ka)
- [x] PID
- [X] Use ContinuousWrap
- [X] Control mode (motion magic, voltage, torque)
- [X] Enable/Disable FOC (only talonfx)

SteerSimulation:
-----------------------
- [X] Moment of inertia
- [X] DCMotor

Drive:
-----------------------
- [x] Motor ID
- [x] Inverted
- [X] Neutral Mode
- [X] Current Limit
- [X] Gear Ratio (should use RotorToSensorRatio)
- [ ] FF (ks, kv, ka)
- [ ] PID
- [ ] Control mode (motion magic, voltage, torque)
- [ ] Enable/Disable FOC (only talonfx)

DriveSimulation:
-----------------------
 - [X] Moment of inertia
 - [X] DCMotor
