package frc.robot.statemachine.funnelstatehandler;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.VelocityRoller;
import frc.utils.time.TimeUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import java.util.function.Supplier;

public class FunnelStateHandler {

	private final VelocityRoller magazine;
	private final Roller conveyor;
	private final Roller upperRoller;

	private final IDigitalInput ballSensor;
	private final DigitalInputInputsAutoLogged sensorInputsAutoLogged;

	private final LoggedNetworkNumber magazineCalibrationVoltage;
	private final LoggedNetworkNumber conveyorCalibrationVoltage;
	private final LoggedNetworkNumber upperRollerCalibrationVoltage;

	private final String logPath;
	protected FunnelState currentState;

	private final Supplier<Double> lastBallThrownTimestamp;

	public FunnelStateHandler(
		VelocityRoller magazine,
		Roller conveyor,
		Roller upperRoller,
		String logPath,
		IDigitalInput ballSensor,
		Supplier<Double> lastBallThrownTimestamp
	) {
		this.magazine = magazine;
		this.conveyor = conveyor;
		this.upperRoller = upperRoller;

		this.ballSensor = ballSensor;
		this.sensorInputsAutoLogged = new DigitalInputInputsAutoLogged();

		this.magazineCalibrationVoltage = new LoggedNetworkNumber("Tunable/MagazineVoltage", 0);
		this.conveyorCalibrationVoltage = new LoggedNetworkNumber("Tunable/ConveyorVoltage", 0);
		this.upperRollerCalibrationVoltage = new LoggedNetworkNumber("Tunable/UpperRollerVoltage", 0);

		this.logPath = logPath + "/FunnelStateHandler";
		this.currentState = FunnelState.STOP;

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
				conveyor.getCommandsBuilder().setVoltage(FunnelState.ROLL_UNTIL_SENSOR.getConveyorVoltage()),
				upperRoller.getCommandsBuilder().setVoltage(FunnelState.ROLL_UNTIL_SENSOR.getUpperRollerVoltage())
			).until(this::isBallAtSensor),
			new ParallelCommandGroup(
				magazine.getCommandsBuilder().stop(),
				conveyor.getCommandsBuilder().stop(),
				upperRoller.getCommandsBuilder().stop()
			)
		);
	}

	private Command outtake() {
		return new ParallelCommandGroup(
			magazine.getCommandsBuilder().stop(),
			conveyor.getCommandsBuilder().setVoltage(FunnelState.OUTTAKE.getConveyorVoltage()),
			upperRoller.getCommandsBuilder().setVoltage(FunnelState.OUTTAKE.getUpperRollerVoltage())
		);
	}

	private Command outtakeShoot() {
		return new ParallelCommandGroup(
			magazine.getCommandsBuilder().setVelocity(FunnelState.OUTTAKE_SHOOT.getMagazineVelocity()),
			conveyor.getCommandsBuilder().setVoltage(FunnelState.OUTTAKE_SHOOT.getConveyorVoltage()),
			upperRoller.getCommandsBuilder().setVoltage(FunnelState.OUTTAKE_SHOOT.getUpperRollerVoltage())
		);
	}

	private Command preShoot() {
		return new ParallelCommandGroup(
			magazine.getCommandsBuilder().setVelocity(FunnelState.PRE_SHOOT.getMagazineVelocity()),
			conveyor.getCommandsBuilder().setVoltage(FunnelState.PRE_SHOOT.getConveyorVoltage()),
			upperRoller.getCommandsBuilder().setVoltage(FunnelState.PRE_SHOOT.getUpperRollerVoltage())
		);
	}

	private Command shoot() {
		return new ParallelCommandGroup(
			magazine.getCommandsBuilder().setVelocity(FunnelState.SHOOT.getMagazineVelocity()),
			new SequentialCommandGroup(
				new ParallelCommandGroup(
					conveyor.getCommandsBuilder().setVoltage(FunnelState.PRE_SHOOT.getConveyorVoltage()),
					upperRoller.getCommandsBuilder().setVoltage(FunnelState.PRE_SHOOT.getUpperRollerVoltage())
				).withTimeout(FunnelConstants.TIME_FOR_MAGAZINE_TO_ACCELERATE_SECONDS),
				new ParallelCommandGroup(
					conveyor.getCommandsBuilder().setVoltage(FunnelState.SHOOT.getConveyorVoltage()),
					upperRoller.getCommandsBuilder().setVoltage(FunnelState.SHOOT.getUpperRollerVoltage())
				).withTimeout(FunnelConstants.TIME_FOR_CONVEYOR_TO_ACCELERATE_SECONDS),
				new RepeatCommand(
					new SequentialCommandGroup(
						new ParallelCommandGroup(
							conveyor.getCommandsBuilder().setVoltage(FunnelState.SHOOT.getConveyorVoltage()),
							upperRoller.getCommandsBuilder().setVoltage(FunnelState.SHOOT.getUpperRollerVoltage())
						).until(
							() -> (TimeUtil.getCurrentTimeSeconds() - lastBallThrownTimestamp.get()
								> FunnelConstants.TIME_BETWEEN_BALLS_TO_DETECT_JAM_SECONDS)
						),
						new ParallelCommandGroup(
							conveyor.getCommandsBuilder().setVoltage(FunnelState.OUTTAKE_SHOOT.getConveyorVoltage()),
							upperRoller.getCommandsBuilder().setVoltage(FunnelState.OUTTAKE_SHOOT.getUpperRollerVoltage())
						).withTimeout(FunnelConstants.TIME_FOR_CONVEYOR_OUTTAKE_DURING_JAM_SECONDS),
						new ParallelCommandGroup(
							conveyor.getCommandsBuilder().setVoltage(FunnelState.SHOOT.getConveyorVoltage()),
							upperRoller.getCommandsBuilder().setVoltage(FunnelState.SHOOT.getUpperRollerVoltage())
						).withTimeout(FunnelConstants.TIME_FOR_CONVEYOR_TO_IGNORE_STUCK_SECONDS)
					)
				)
			)
		);
	}

	private Command stop() {
		return new ParallelCommandGroup(
			magazine.getCommandsBuilder().stop(),
			conveyor.getCommandsBuilder().stop(),
			upperRoller.getCommandsBuilder().stop()
		);
	}

	private Command calibration() {
		return new ParallelCommandGroup(
			magazine.getCommandsBuilder().setVoltage(() -> magazineCalibrationVoltage.get()),
			conveyor.getCommandsBuilder().setVoltage(() -> conveyorCalibrationVoltage.get()),
			upperRoller.getCommandsBuilder().setVoltage(() -> upperRollerCalibrationVoltage.get())
		);
	}

	public void periodic() {
		ballSensor.updateInputs(sensorInputsAutoLogged);

		Logger.recordOutput(logPath + "/CurrentState", currentState);
		Logger.processInputs(logPath, sensorInputsAutoLogged);
	}

}
