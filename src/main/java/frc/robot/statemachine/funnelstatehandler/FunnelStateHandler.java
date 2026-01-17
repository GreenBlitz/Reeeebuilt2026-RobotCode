package frc.robot.statemachine.funnelstatehandler;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.subsystems.roller.Roller;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class FunnelStateHandler {

	private final Roller omni;

	private final IDigitalInput sensor;
	private final DigitalInputInputsAutoLogged sensorInputsAutoLogged;

	private final Roller belly;

	private final String logPath;

	private final LoggedNetworkNumber omniCalibrationVoltage;

	private final LoggedNetworkNumber bellyCalibrationVoltage;

	protected FunnelState currentState;

	public FunnelStateHandler(Roller omni, Roller belly, String logPath, IDigitalInput sensor) {
		this.omni = omni;
		this.belly = belly;
		this.sensor = sensor;
		this.logPath = logPath + "/FunnelStateHandler";
		this.currentState = FunnelState.STOP;
		this.omniCalibrationVoltage = new LoggedNetworkNumber("Tunable/OmniPower", 0);
		this.bellyCalibrationVoltage = new LoggedNetworkNumber("Tunable/BellyPower", 0);
		this.sensorInputsAutoLogged = new DigitalInputInputsAutoLogged();
		Logger.recordOutput(logPath + "/CurrentState", currentState.name());
		sensor.updateInputs(sensorInputsAutoLogged);
	}

	public Command setState(FunnelState state) {
		Command command = switch (state) {
			case DRIVE -> drive();
			case SHOOT -> shoot();
			case SHOOT_WHILE_INTAKE -> shootWhileIntake();
			case INTAKE -> intake();
			case STOP -> stop();
			case CALIBRATION -> calibration();
		};
		return new ParallelCommandGroup(
			new InstantCommand(() -> Logger.recordOutput(logPath + "/CurrentState", state.name())),
			new InstantCommand(() -> currentState = state),
			command
		);
	}

	public boolean isBallAtSensor() {
		return sensorInputsAutoLogged.debouncedValue;
	}

	private Command drive() {
		return new ParallelCommandGroup(omni.getCommandsBuilder().stop(), belly.getCommandsBuilder().stop());
	}

	private Command shoot() {
		return new ParallelCommandGroup(
			omni.getCommandsBuilder().setVoltage(FunnelState.SHOOT.getOmniVoltage()),
			belly.getCommandsBuilder().setVoltage(FunnelState.SHOOT.getBellyVoltage())
		);
	}

	private Command shootWhileIntake() {
		return new ParallelCommandGroup(
			omni.getCommandsBuilder().setVoltage(FunnelState.SHOOT_WHILE_INTAKE.getOmniVoltage()),
			belly.getCommandsBuilder().setVoltage(FunnelState.SHOOT_WHILE_INTAKE.getBellyVoltage())
		);
	}

	private Command intake() {
		return new SequentialCommandGroup(
			new ParallelCommandGroup(
				omni.getCommandsBuilder().setVoltage(FunnelState.INTAKE.getOmniVoltage()),
				belly.getCommandsBuilder().setVoltage(FunnelState.INTAKE.getBellyVoltage())
			).until(() -> this.isBallAtSensor()),
			new ParallelCommandGroup(omni.getCommandsBuilder().stop(), belly.getCommandsBuilder().stop())
		);
	}

	private Command stop() {
		return new ParallelCommandGroup(omni.getCommandsBuilder().stop(), belly.getCommandsBuilder().stop());
	}

	private Command calibration() {
		return new ParallelCommandGroup(
			omni.getCommandsBuilder().setVoltage(() -> omniCalibrationVoltage.get()),
			belly.getCommandsBuilder().setVoltage(() -> bellyCalibrationVoltage.get())
		);
	}

	public void periodic() {
		sensor.updateInputs(sensorInputsAutoLogged);
		Logger.processInputs(logPath, sensorInputsAutoLogged);
	}

}
