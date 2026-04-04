package frc.robot.statemachine.funnelstatehandler;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;
import frc.robot.statemachine.StateMachineConstants;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.VelocityRoller;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.function.Supplier;

public class FunnelStateHandler {

	private final VelocityRoller magazine;

	private final IDigitalInput ballSensor;
	private final DigitalInputInputsAutoLogged sensorInputsAutoLogged;

	private final Roller conveyor;

	private final String logPath;

	private final LoggedNetworkNumber magazineCalibrationVoltage;

	private final LoggedNetworkNumber conveyorCalibrationVoltage;

	protected FunnelState currentState;

	private final Supplier<Double> lastBallThrownTimestamp;

	public FunnelStateHandler(VelocityRoller magazine, Roller conveyor, String logPath, IDigitalInput ballSensor, Supplier<Double> lastBallThrownTimestamp) {
		this.magazine = magazine;
		this.conveyor = conveyor;
		this.logPath = logPath + "/FunnelStateHandler";
		this.currentState = FunnelState.STOP;
		this.magazineCalibrationVoltage = new LoggedNetworkNumber("Tunable/MagazineVoltage", 0);
		this.conveyorCalibrationVoltage = new LoggedNetworkNumber("Tunable/ConveyorVoltage", 0);
		this.ballSensor = ballSensor;
		this.sensorInputsAutoLogged = new DigitalInputInputsAutoLogged();
		this.lastBallThrownTimestamp = lastBallThrownTimestamp;
		Logger.recordOutput(logPath + "/CurrentState", currentState.name());
	}

	public Command setState(FunnelState state) {
		Command command = switch (state) {
			case ROLL_UNTIL_SENSOR -> rollUntilSensor();
			case SHOOT -> shoot();
			case STOP -> stop();
			case OUTTAKE -> outtake();
			case PRE_SHOOT -> preShoot();
			case OUTTAKE_SHOOT -> outtakeShoot();
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

	private Command outtakeShoot() {
		return new ParallelCommandGroup(
			magazine.getCommandsBuilder().setVelocity(FunnelState.OUTTAKE_SHOOT.getMagazineVelocity()),
			conveyor.getCommandsBuilder().setVoltage(FunnelState.OUTTAKE_SHOOT.getConveyorVoltage())
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
				conveyor.getCommandsBuilder()
					.setVoltage(FunnelState.PRE_SHOOT.getConveyorVoltage())
					.withTimeout(StateMachineConstants.TIME_FOR_MAGAZINE_TO_ACCELERATE_SECONDS),
					conveyor.getCommandsBuilder().setVoltage(FunnelState.SHOOT.getConveyorVoltage()).withTimeout(FunnelConstants.CONVEYOR_STARTING_ACCELERATION_DELAY_SECONDS),
					new RepeatCommand(
							new SequentialCommandGroup(
									conveyor.getCommandsBuilder().setVoltage(FunnelState.SHOOT.getConveyorVoltage())
											.until(() -> (TimeUtil.getCurrentTimeSeconds() - lastBallThrownTimestamp.get() > FunnelConstants.TIME_BETWEEN_BALLS_TO_CONSIDER_STUCK_SECONDS)),
									conveyor.getCommandsBuilder().setVoltage(FunnelState.OUTTAKE_SHOOT.getConveyorVoltage()).withTimeout(FunnelConstants.CONVEYOR_OUTTAKE_DURING_STUCK_SECONDS),
									conveyor.getCommandsBuilder().setVoltage(FunnelState.SHOOT.getConveyorVoltage()).withTimeout(FunnelConstants.TIME_FOR_CONVEYOR_TO_IGNORE_STUCK)
							)
					)
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
		ballSensor.updateInputs(sensorInputsAutoLogged);

		Logger.recordOutput(logPath + "/CurrentState", currentState);
		Logger.processInputs(logPath, sensorInputsAutoLogged);
	}

}
