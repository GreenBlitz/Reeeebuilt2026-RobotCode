package frc.robot.statemachine.funnelstatehandler;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.statemachine.StateMachineConstants;
import frc.robot.subsystems.roller.Roller;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class FunnelStateHandler {

	private final Roller train;

	private final IDigitalInput sensor;
	private final DigitalInputInputsAutoLogged sensorInputsAutoLogged;

	private final Roller belly;

	private final String logPath;

	private final LoggedNetworkNumber trainCalibrationVoltage;

	private final LoggedNetworkNumber bellyCalibrationVoltage;

	protected FunnelState currentState;

	public FunnelStateHandler(Roller train, Roller belly, String logPath, IDigitalInput sensor) {
		this.train = train;
		this.belly = belly;
		this.sensor = sensor;
		this.logPath = logPath + "/FunnelStateHandler";
		this.currentState = FunnelState.STOP;
		this.trainCalibrationVoltage = new LoggedNetworkNumber("Tunable/TrainVoltage", 0);
		this.bellyCalibrationVoltage = new LoggedNetworkNumber("Tunable/BellyVoltage", 0);
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
		return new ParallelCommandGroup(train.getCommandsBuilder().stop(), belly.getCommandsBuilder().stop());
	}

	private Command shoot() {
		return new ParallelCommandGroup(
			train.getCommandsBuilder().setVoltage(FunnelState.SHOOT.getTrainVoltage()),
			new SequentialCommandGroup(
				new WaitCommand(StateMachineConstants.TIME_FOR_TRAIN_TO_ACCELERATE_SECONDS),
				belly.getCommandsBuilder().setVoltage(FunnelState.SHOOT.getBellyVoltage())
			)
		);
	}

	private Command shootWhileIntake() {
		return new ParallelCommandGroup(
			train.getCommandsBuilder().setVoltage(FunnelState.SHOOT_WHILE_INTAKE.getTrainVoltage()),
			new SequentialCommandGroup(
				new WaitCommand(StateMachineConstants.TIME_FOR_TRAIN_TO_ACCELERATE_SECONDS),
				belly.getCommandsBuilder().setVoltage(FunnelState.SHOOT_WHILE_INTAKE.getBellyVoltage())
			)
		);
	}

	private Command intake() {
		return new SequentialCommandGroup(
			new ParallelCommandGroup(
				belly.getCommandsBuilder().setVoltage(FunnelState.INTAKE.getBellyVoltage()),
				train.getCommandsBuilder().setVoltage(FunnelState.INTAKE.getTrainVoltage())
			).until(() -> this.isBallAtSensor()),
			new ParallelCommandGroup(train.getCommandsBuilder().stop(), belly.getCommandsBuilder().stop())
		);
	}

	private Command stop() {
		return new ParallelCommandGroup(train.getCommandsBuilder().stop(), belly.getCommandsBuilder().stop());
	}

	private Command calibration() {
		return new ParallelCommandGroup(
			train.getCommandsBuilder().setVoltage(() -> trainCalibrationVoltage.get()),
			belly.getCommandsBuilder().setVoltage(() -> bellyCalibrationVoltage.get())
		);
	}

	public void periodic() {
		sensor.updateInputs(sensorInputsAutoLogged);
		Logger.processInputs(logPath, sensorInputsAutoLogged);
	}

}
