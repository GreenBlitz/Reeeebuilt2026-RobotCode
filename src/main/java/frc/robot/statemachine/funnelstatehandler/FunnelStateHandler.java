package frc.robot.statemachine.funnelstatehandler;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.statemachine.StateMachineConstants;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.VelocityRoller;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class FunnelStateHandler {

	private final VelocityRoller train;

	private final DigitalInputInputsAutoLogged sensorInputsAutoLogged;

	private final Roller belly;

	private final String logPath;

	private final LoggedNetworkNumber trainCalibrationVoltage;

	private final LoggedNetworkNumber bellyCalibrationVoltage;

	protected FunnelState currentState;

	public FunnelStateHandler(VelocityRoller train, Roller belly, String logPath) {
		this.train = train;
		this.belly = belly;
		this.logPath = logPath + "/FunnelStateHandler";
		this.currentState = FunnelState.STOP;
		this.trainCalibrationVoltage = new LoggedNetworkNumber("Tunable/TrainVoltage", 0);
		this.bellyCalibrationVoltage = new LoggedNetworkNumber("Tunable/BellyVoltage", 0);
		this.sensorInputsAutoLogged = new DigitalInputInputsAutoLogged();
		Logger.recordOutput(logPath + "/CurrentState", currentState.name());
	}

	public Command setState(FunnelState state) {
		Command command = switch (state) {
			case NEUTRAL -> neutral();
			case SHOOT -> shoot();
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

	private Command neutral() {
		return new ParallelCommandGroup(train.getCommandsBuilder().stop(), belly.getCommandsBuilder().stop());
	}

	private Command shoot() {
		return new ParallelCommandGroup(
			train.getCommandsBuilder().setVelocity(FunnelState.SHOOT.getTrainVelocity()),
			new SequentialCommandGroup(
				new WaitCommand(StateMachineConstants.TIME_FOR_TRAIN_TO_ACCELERATE_SECONDS),
				belly.getCommandsBuilder().setVoltage(FunnelState.SHOOT.getBellyVoltage())
			)
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
		Logger.recordOutput(logPath + "/CurrentState", currentState);
		Logger.processInputs(logPath, sensorInputsAutoLogged);
	}

}
