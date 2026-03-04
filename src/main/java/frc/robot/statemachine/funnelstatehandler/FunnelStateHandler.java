package frc.robot.statemachine.funnelstatehandler;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.statemachine.StateMachineConstants;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.VelocityRoller;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

public class FunnelStateHandler {

	private final VelocityRoller train;

	private final IDigitalInput ballSensor;
	private final DigitalInputInputsAutoLogged sensorInputsAutoLogged;

	private final Roller conveyor;

	private final String logPath;

	private final LoggedNetworkNumber trainCalibrationVoltage;

	private final LoggedNetworkNumber conveyorCalibrationVoltage;

	protected FunnelState currentState;

	public FunnelStateHandler(VelocityRoller train, Roller conveyor, String logPath, IDigitalInput ballSensor) {
		this.train = train;
		this.conveyor = conveyor;
		this.logPath = logPath + "/FunnelStateHandler";
		this.currentState = FunnelState.STOP;
		this.trainCalibrationVoltage = new LoggedNetworkNumber("Tunable/TrainVoltage", 0);
		this.conveyorCalibrationVoltage = new LoggedNetworkNumber("Tunable/ConveyorVoltage", 0);
		this.ballSensor = ballSensor;
		this.sensorInputsAutoLogged = new DigitalInputInputsAutoLogged();
		Logger.recordOutput(logPath + "/CurrentState", currentState.name());
	}

	public Command setState(FunnelState state) {
		Command command = switch (state) {
			case ROLL_UNTIL_SENSOR -> rollUntilSensor();
			case SHOOT -> shoot();
			case STOP -> stop();
			case OUTTAKE -> outtake();
			case PRE_SHOOT -> preShoot();
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

	private Command rollUntilSensor() {
		return new SequentialCommandGroup(
			new ParallelCommandGroup(
				train.getCommandsBuilder().setVelocity(FunnelState.ROLL_UNTIL_SENSOR.getTrainVelocity()),
				conveyor.getCommandsBuilder().setVoltage(FunnelState.ROLL_UNTIL_SENSOR.getConveyorVoltage())
			).until(this::isBallAtSensor),
			new ParallelCommandGroup(train.getCommandsBuilder().stop(), conveyor.getCommandsBuilder().stop())
		);
	}

	private Command outtake() {
		return new ParallelCommandGroup(
			train.getCommandsBuilder().stop(),
			conveyor.getCommandsBuilder().setVoltage(FunnelState.OUTTAKE.getConveyorVoltage())
		);
	}

	private Command preShoot() {
		return new ParallelCommandGroup(
			train.getCommandsBuilder().setVelocity(FunnelState.PRE_SHOOT.getTrainVelocity()),
			conveyor.getCommandsBuilder().setVoltage(FunnelState.PRE_SHOOT.getConveyorVoltage())
		);
	}

	private Command shoot() {
		return new ParallelCommandGroup(
			train.getCommandsBuilder().setVelocity(FunnelState.SHOOT.getTrainVelocity()),
			new SequentialCommandGroup(
				new WaitCommand(StateMachineConstants.TIME_FOR_TRAIN_TO_ACCELERATE_SECONDS),
				conveyor.getCommandsBuilder().setVoltage(FunnelState.SHOOT.getConveyorVoltage())
			)
		);
	}

	private Command stop() {
		return new ParallelCommandGroup(train.getCommandsBuilder().stop(), conveyor.getCommandsBuilder().stop());
	}

	private Command calibration() {
		return new ParallelCommandGroup(
			train.getCommandsBuilder().setVoltage(() -> trainCalibrationVoltage.get()),
			conveyor.getCommandsBuilder().setVoltage(() -> conveyorCalibrationVoltage.get())
		);
	}

	public void periodic() {
		ballSensor.updateInputs(sensorInputsAutoLogged);

		Logger.recordOutput(logPath + "/CurrentState", currentState);
		Logger.processInputs(logPath, sensorInputsAutoLogged);
	}

}
