package frc.robot.statemachine.intakestatehandler;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.arm.CurrentControlArm;
import frc.robot.subsystems.constants.fourBar.FourBarConstants;
import frc.robot.subsystems.roller.Roller;
import frc.utils.LoggedNetworkRotation2d;
import frc.utils.driverstation.DriverStationUtil;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import java.util.Set;
import java.util.function.BooleanSupplier;

public class IntakeStateHandler {

	private final CurrentControlArm fourBar;
	private final Roller rollers;
	private boolean hasFourBarBeenReset;
	private final String logPath;
	private final LoggedNetworkNumber rollersCalibrationPower = new LoggedNetworkNumber("Tunable/IntakeRollerPower");
	private final LoggedNetworkRotation2d fourBarCalibrationPosition = new LoggedNetworkRotation2d("Tunable/FourBarPosition", new Rotation2d());

	private IntakeState currentState;
	private BooleanSupplier isOpenFourBarLocked;
	private BooleanSupplier isCloseFourBarHarder;

	public IntakeStateHandler(CurrentControlArm fourBar, Roller rollers, String logPath) {
		this.fourBar = fourBar;
		this.rollers = rollers;
		this.hasFourBarBeenReset = Robot.ROBOT_TYPE.isSimulation();
		this.logPath = logPath + "/IntakeStateHandler";
		this.currentState = IntakeState.STAY_IN_PLACE;
		this.isOpenFourBarLocked = () -> false;
		this.isCloseFourBarHarder = () -> false;
	}

	public void setIntakeButtonsSuppliers(BooleanSupplier openFourBarLocked, BooleanSupplier closeFourBarHarder) {
		this.isOpenFourBarLocked = () -> openFourBarLocked.getAsBoolean() || DriverStationUtil.isAutonomous();
		this.isCloseFourBarHarder = () -> closeFourBarHarder.getAsBoolean() || DriverStationUtil.isAutonomous();
	}

	public Command calibration() {
		return new ParallelCommandGroup(
			fourBar.getCommandsBuilder().setTargetPosition(fourBarCalibrationPosition::get),
			rollers.getCommandsBuilder().setPower(rollersCalibrationPower::get)
		);
	}

	public Command stayInPlace() {
		return new ParallelCommandGroup(fourBar.getCommandsBuilder().stayInPlace(), rollers.getCommandsBuilder().stop());
	}

	public Command resetFourBar() {
		return fourBar.getCommandsBuilder().setVoltageWithoutLimit(FourBarConstants.FOUR_BAR_RESET_VOLTAGE, () -> hasFourBarBeenReset());
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
						Set.of(fourBar, rollers)
					)
				)
		);
	}

	public Command intake() {
		return new ParallelCommandGroup(openFourBar(), rollers.getCommandsBuilder().setPower(IntakeState.INTAKE.getIntakePower()));
	}

	public Command outtake() {
		return new ParallelCommandGroup(openFourBar(), rollers.getCommandsBuilder().setPower(IntakeState.OUTTAKE.getIntakePower()));
	}

	public Command close() {
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				new ParallelDeadlineGroup(
					new SequentialCommandGroup(
						fourBar.getCommandsBuilder()
							.setCurrentWithoutLimit(FourBarConstants.STRONG_CLOSE_CURRENT_AMP)
							.withTimeout(FourBarConstants.STRONG_CLOSE_TIME_SECONDS),
						fourBar.getCommandsBuilder()
							.setVoltageWithoutLimit(
								FourBarConstants.CLOSE_VOLTAGE,
								() -> fourBar.getCurrent() > FourBarConstants.COLLISION_STALL_CURRENT
							),
							fourBar.getCommandsBuilder()
									.setCurrentWithoutLimit(FourBarConstants.RELAXED_CLOSE_CURRENT_AMP)
									.withTimeout(FourBarConstants.RELAXED_CLOSE_TIME_SECONDS)
					),

					rollers.getCommandsBuilder().setPower(IntakeState.INTAKE.getIntakePower())
				),
				new ParallelCommandGroup(

						fourBar.getCommandsBuilder()
							.setCurrentWithoutLimit(
								() -> isCloseFourBarHarder.getAsBoolean()
									? FourBarConstants.RELAXED_CLOSE_CURRENT_AMP
									: FourBarConstants.HOLD_CLOSE_CURRENT_AMP
							),
					rollers.getCommandsBuilder().setPower(IntakeState.CLOSED.getIntakePower())
				)
			)
		);
	}

	public Command setState(IntakeState intakeState) {
		return new ParallelCommandGroup(switch (intakeState) {
			case CALIBRATION -> calibration();
			case STAY_IN_PLACE -> stayInPlace();
			case INTAKE -> intake();
			case OUTTAKE -> outtake();
			case RESET_FOUR_BAR -> resetFourBar();
			case CLOSED -> close();
		},
			new InstantCommand(() -> Logger.recordOutput(logPath + "/CurrentState", intakeState.name())),
			new InstantCommand(() -> currentState = intakeState)
		);
	}

	private Command openFourBar() {
		return new SequentialCommandGroup(
			// fourBar.getCommandsBuilder()
//				.setVoltageWithoutLimit(FourBarConstants.OPEN_VOLTAGE, () -> fourBar.getCurrent() > FourBarConstants.COLLISION_STALL_CURRENT),
			fourBar.getCommandsBuilder()
				.setCurrentWithoutLimit(FourBarConstants.STRONG_OPEN_CURRENT_AMP)
				.withTimeout(FourBarConstants.RELAXED_OPEN_TIME_SECONDS),
			fourBar.getCommandsBuilder()
				.setCurrentWithoutLimit(
					() -> isOpenFourBarLocked.getAsBoolean() ? FourBarConstants.RELAXED_OPEN_CURRENT_AMP : FourBarConstants.HOLD_OPEN_CURRENT_AMP
				)
		);
	}

	public void periodic() {
		Logger.recordOutput(logPath + "/IsOpenFourBarLocked", isOpenFourBarLocked.getAsBoolean());
		Logger.recordOutput(logPath + "/isCloseFourBarHarder", isCloseFourBarHarder.getAsBoolean());

		if (!hasFourBarBeenReset() && fourBar.getCurrent() > FourBarConstants.CURRENT_THRESHOLD_TO_RESET_POSITION) {
			hasFourBarBeenReset = true;
		}

		if (FourBarConstants.MAXIMUM_POSITION.getRadians() < fourBar.getPosition().getRadians() && !hasFourBarBeenReset()) {
			fourBar.setPosition(FourBarConstants.MAXIMUM_POSITION);
		}

		Logger.recordOutput(logPath + "/HasFourBarBeenReset", hasFourBarBeenReset());
		Logger.recordOutput(logPath + "/CurrentState", currentState);
	}

	public boolean hasFourBarBeenReset() {
		return hasFourBarBeenReset;
	}

	public IntakeState getCurrentState() {
		return currentState;
	}

}
