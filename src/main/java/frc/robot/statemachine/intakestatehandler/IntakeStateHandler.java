package frc.robot.statemachine.intakestatehandler;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.arm.CurrentControlArm;
import frc.robot.subsystems.constants.pivot.PivotConstants;
import frc.robot.subsystems.roller.Roller;
import frc.utils.LoggedNetworkRotation2d;
import frc.utils.driverstation.DriverStationUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import java.util.Set;
import java.util.function.BooleanSupplier;

public class IntakeStateHandler {

	private final CurrentControlArm pivot;
	private final Roller rollers;
	private boolean hasPivotBeenReset;
	private final String logPath;
	private final LoggedNetworkNumber rollersCalibrationPower = new LoggedNetworkNumber("Tunable/IntakeRollerPower");
	private final LoggedNetworkRotation2d pivotCalibrationPosition = new LoggedNetworkRotation2d("Tunable/PivotPosition", new Rotation2d());

	private IntakeState currentState;
	private BooleanSupplier isOpenPivotHarder;

	public IntakeStateHandler(CurrentControlArm pivot, Roller rollers, String logPath) {
		this.pivot = pivot;
		this.rollers = rollers;
		this.hasPivotBeenReset = Robot.ROBOT_TYPE.isSimulation();
		this.logPath = logPath + "/IntakeStateHandler";
		this.currentState = IntakeState.STAY_IN_PLACE;
		this.isOpenPivotHarder = () -> false;
	}

	public void setIntakeButtonsSuppliers(BooleanSupplier openPivotLocked) {
		this.isOpenPivotHarder = () -> openPivotLocked.getAsBoolean() || DriverStationUtil.isAutonomous();
	}

	public Command calibration() {
		return new ParallelCommandGroup(
			pivot.getCommandsBuilder().setTargetPosition(pivotCalibrationPosition::get),
			rollers.getCommandsBuilder().setPower(rollersCalibrationPower::get)
		);
	}

	public Command stayInPlace() {
		return new ParallelCommandGroup(pivot.getCommandsBuilder().stayInPlace(), rollers.getCommandsBuilder().stop());
	}

	public Command resetPivot() {
		return pivot.getCommandsBuilder().setVoltageWithoutLimit(PivotConstants.PIVOT_RESET_VOLTAGE, () -> hasPivotBeenReset());
	}

	public Command toggleState() {
		return new InstantCommand(
			() -> CommandScheduler.getInstance()
				.schedule(
					new DeferredCommand(
						() -> new ConditionalCommand(
							setState(IntakeState.INTAKE),
							setState(IntakeState.CLOSED),
							() -> currentState != IntakeState.INTAKE
						),
						Set.of(pivot, rollers)
					)
				)
		);
	}

	public Command intake() {
		return new ParallelCommandGroup(openPivot(), rollers.getCommandsBuilder().setPower(IntakeState.INTAKE.getIntakePower()));
	}

	public Command outtake() {
		return new ParallelCommandGroup(openPivot(), rollers.getCommandsBuilder().setPower(IntakeState.OUTTAKE.getIntakePower()));
	}

	public Command close() {
		return new ParallelCommandGroup(
			pivot.getCommandsBuilder().setTargetPosition(IntakeState.CLOSED.getPivotPosition()),
			new SequentialCommandGroup(
				rollers.getCommandsBuilder().setPower(IntakeState.INTAKE.getIntakePower()).withTimeout(0.5),
				rollers.getCommandsBuilder().setPower(IntakeState.CLOSED.getIntakePower())
			)
		);
	}

	public Command slowClose() {
		return new ParallelDeadlineGroup(
			pivot.getCommandsBuilder()
				.setVoltageWithoutLimit(
					PivotConstants.SLOW_CLOSE_VOLTAGE,
					() -> pivot.isPastPosition(PivotConstants.PIVOT_POSITION_FOR_SLOW_CLOSE)
				),
			rollers.getCommandsBuilder().setPower(IntakeState.SLOW_CLOSE.getIntakePower())

		);
	}

	public Command setState(IntakeState intakeState) {
		return new ParallelCommandGroup(switch (intakeState) {
			case CALIBRATION -> calibration();
			case STAY_IN_PLACE -> stayInPlace();
			case INTAKE -> intake();
			case OUTTAKE -> outtake();
			case RESET_PIVOT -> resetPivot();
			case CLOSED -> close();
			case SLOW_CLOSE -> slowClose();
		},
			new InstantCommand(() -> Logger.recordOutput(logPath + "/CurrentState", intakeState.name())),
			new InstantCommand(() -> currentState = intakeState)
		);
	}

	private Command openPivot() {
		return new SequentialCommandGroup(
			pivot.getCommandsBuilder().setTargetPosition(IntakeState.INTAKE.getPivotPosition()),
			pivot.getCommandsBuilder()
				.setCurrentWithoutLimit(
					() -> isOpenPivotHarder.getAsBoolean() ? PivotConstants.COLLISION_OPEN_CURRENT_AMP : PivotConstants.HOLD_OPEN_CURRENT_AMP
				)
		);
	}

	public void periodic() {
		Logger.recordOutput(logPath + "/IsOpenPivotHarder", isOpenPivotHarder.getAsBoolean());

		if (!hasPivotBeenReset() && pivot.getCurrent() > PivotConstants.CURRENT_THRESHOLD_TO_RESET_POSITION) {
			hasPivotBeenReset = true;
		}

		if (PivotConstants.MAXIMUM_POSITION.getRadians() < pivot.getPosition().getRadians() && !hasPivotBeenReset()) {
			pivot.setPosition(PivotConstants.MAXIMUM_POSITION);
		}

		Logger.recordOutput(logPath + "/HasPivotBeenReset", hasPivotBeenReset());
		Logger.recordOutput(logPath + "/CurrentState", currentState);
	}

	public boolean hasPivotBeenReset() {
		return hasPivotBeenReset;
	}

	public IntakeState getCurrentState() {
		return currentState;
	}

}
