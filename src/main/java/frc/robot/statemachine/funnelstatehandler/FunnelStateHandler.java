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

	private final VelocityRoller magazine;

	private final IDigitalInput ballSensor;
	private final DigitalInputInputsAutoLogged ballSensorInputs;

	private final Roller conveyor;

	private final String logPath;

	private final LoggedNetworkNumber magazineCalibrationVoltage;

	private final LoggedNetworkNumber conveyorCalibrationVoltage;

	protected FunnelState currentState;

	public FunnelStateHandler(VelocityRoller magazine, Roller conveyor, String logPath, IDigitalInput ballSensor) {
		this.magazine = magazine;
		this.conveyor = conveyor;
		this.logPath = logPath + "/FunnelStateHandler";
		this.currentState = FunnelState.STOP;
		this.magazineCalibrationVoltage = new LoggedNetworkNumber("Tunable/MagazineVoltage", 0);
		this.conveyorCalibrationVoltage = new LoggedNetworkNumber("Tunable/ConveyorVoltage", 0);
		this.ballSensor = ballSensor;
		this.ballSensorInputs = new DigitalInputInputsAutoLogged();
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
		return ballSensorInputs.debouncedValue;
	}

	private Command rollUntilSensor() {
		return new SequentialCommandGroup(
			new ParallelCommandGroup(
				magazine.getCommandsBuilder().setVelocity(FunnelState.ROLL_UNTIL_SENSOR.getMagazineVelocity()),
				conveyor.getCommandsBuilder().setVoltage(FunnelState.ROLL_UNTIL_SENSOR.getConveyorVoltage())
			).until(this::isBallAtSensor),
			new ParallelCommandGroup(magazine.getCommandsBuilder().stop(), conveyor.getCommandsBuilder().stop())
		);
	}

	private Command outtake() {
		return new ParallelCommandGroup(
			magazine.getCommandsBuilder().stop(),
			conveyor.getCommandsBuilder().setVoltage(FunnelState.OUTTAKE.getConveyorVoltage())
		);
	}

	private Command preShoot() {
		return new ParallelCommandGroup(
			magazine.getCommandsBuilder().setVelocity(FunnelState.PRE_SHOOT.getMagazineVelocity()),
			conveyor.getCommandsBuilder().setVoltage(FunnelState.PRE_SHOOT.getConveyorVoltage())
		);
	}

	private Command shoot() {
		return new ParallelCommandGroup(
			magazine.getCommandsBuilder().setVelocity(FunnelState.SHOOT.getMagazineVelocity()),
			new SequentialCommandGroup(
				new WaitCommand(StateMachineConstants.TIME_FOR_MAGAZINE_TO_ACCELERATE_SECONDS),
				conveyor.getCommandsBuilder().setVoltage(FunnelState.SHOOT.getConveyorVoltage())
			)
		);
	}

	private Command stop() {
		return new ParallelCommandGroup(magazine.getCommandsBuilder().stop(), conveyor.getCommandsBuilder().stop());
	}

	private Command calibration() {
		return new ParallelCommandGroup(
			magazine.getCommandsBuilder().setVoltage(() -> magazineCalibrationVoltage.get()),
			conveyor.getCommandsBuilder().setVoltage(() -> conveyorCalibrationVoltage.get())
		);
	}

	public void periodic() {
		ballSensor.updateInputs(ballSensorInputs);

		Logger.recordOutput(logPath + "/CurrentState", currentState);
		Logger.processInputs(logPath + "/BallSensor", ballSensorInputs);
	}

}
