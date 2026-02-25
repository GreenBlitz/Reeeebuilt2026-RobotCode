Modules:
-----------------------
- [x] Enter modules in the order specified in ModuleUtils.ModulePosition

ModuleConstants: (real and simulation)
-----------------------
- [ ] Wheel Diameter
- [x] Coupling Ratio
- [ ] Velocity At 12 Volts
- [x] Modules locations (in meters)

Encoder:
----------------------
- [x] Encoder ID
- [x] Encoder BusChain
- [x] Sensor Range (should be PlusMinusHalf)
- [x] Sensor Direction (should be CounterClockwise in default sds module) //It was Clockwise for us

Steer:
-----------------------
- [x] Motor ID
- [x] Inverted
- [x] Neutral Mode
- [ ] Current Limit
- [x] Gear Ratio (should use RotorToSensorRatio)
- [x] Encoder Usage and ID (should use fuse)
- [ ] FF (ks, kv, ka)
- [ ] PID
- [x] Use ContinuousWrap
- [ ] Control mode (motion magic, voltage, torque)
- [x] Enable/Disable FOC (only talonfx)

SteerSimulation:
-----------------------
- [ ] Moment of inertia
- [ ] DCMotor

Drive:
-----------------------
- [x] Motor ID
- [x] Inverted
- [ ] Neutral Mode
- [ ] Current Limit
- [x] Gear Ratio (should use RotorToSensorRatio)
- [ ] FF (ks, kv, ka)
- [ ] PID
- [ ] Control mode (motion magic, voltage, torque)
- [x] Enable/Disable FOC (only talonfx)

DriveSimulation:
-----------------------
 - [ ] Moment of inertia
 - [ ] DCMotor
