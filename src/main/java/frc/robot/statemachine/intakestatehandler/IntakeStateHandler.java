package frc.robot.statemachine.intakestatehandler;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.subsystems.arm.CurrentControlArm;
import frc.robot.subsystems.constants.fourBar.FourBarConstants;
import frc.robot.subsystems.roller.Roller;
import frc.utils.LoggedNetworkRotation2d;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import java.util.Set;
import java.util.function.BooleanSupplier;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.supplied.SuppliedDigitalInput;

public class IntakeStateHandler {

	private final CurrentControlArm fourBar;
	private final Roller rollers;
	private boolean hasFourBarBeenReset;
	private final String logPath;
	private final LoggedNetworkNumber rollersCalibrationPower = new LoggedNetworkNumber("Tunable/IntakeRollerPower");
	private final LoggedNetworkRotation2d fourBarCalibrationPosition = new LoggedNetworkRotation2d("Tunable/FourBarPosition", new Rotation2d());

	private IntakeState currentState;
	private BooleanSupplier isIntakeButtonHeld = () -> false;
	private SuppliedDigitalInput buttonDigitalInput;
	private final DigitalInputInputsAutoLogged buttonInputs = new DigitalInputInputsAutoLogged();

	public IntakeStateHandler(CurrentControlArm fourBar, Roller rollers, String logPath) {
		this.fourBar = fourBar;
		this.rollers = rollers;
		this.hasFourBarBeenReset = Robot.ROBOT_TYPE.isSimulation();
		this.logPath = logPath + "/IntakeStateHandler";
		this.currentState = IntakeState.STAY_IN_PLACE;
		this.buttonDigitalInput = new SuppliedDigitalInput(isIntakeButtonHeld, new Debouncer(FourBarConstants.DEBOUNCE_TIME_FOR_HOLD));
	}

	public void setIntakeButtonSupplier(BooleanSupplier isIntakeButtonHeld) {
		this.isIntakeButtonHeld = isIntakeButtonHeld;
		this.buttonDigitalInput = new SuppliedDigitalInput(isIntakeButtonHeld, new Debouncer(FourBarConstants.DEBOUNCE_TIME_FOR_HOLD));
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
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				fourBar.getCommandsBuilder()
					.setCurrent(FourBarConstants.INTAKE_OPEN_CURRENT_AMP)
					.until(() -> fourBar.isBehindPosition(IntakeState.INTAKE.getFourBarPosition())),
				fourBar.getCommandsBuilder()
					.setCurrent(() -> buttonInputs.debouncedValue ? FourBarConstants.HOLD_CURRENT_AMP : FourBarConstants.RELAXED_CURRENT_AMP)
			),
			rollers.getCommandsBuilder().setPower(IntakeState.INTAKE.getIntakePower())
		);
	}

	public Command outtake() {
		return new ParallelCommandGroup(
			fourBar.getCommandsBuilder().setTargetPosition(IntakeState.OUTTAKE.getFourBarPosition()),
			rollers.getCommandsBuilder().setPower(IntakeState.OUTTAKE.getIntakePower())
		);
	}

	public Command close() {
		return new ParallelCommandGroup(
			new SequentialCommandGroup(
				fourBar.getCommandsBuilder()
					.setVoltageWithoutLimit(
						FourBarConstants.CLOSE_VOLTAGE,
						() -> fourBar.getCurrent() > FourBarConstants.CLOSE_STALL_CURRENT_AMP
					),
				fourBar.getCommandsBuilder().setCurrent(FourBarConstants.CURRENT_TO_HOLD_INTAKE_CLOSED)
			),
			rollers.getCommandsBuilder().setPower(IntakeState.CLOSED.getIntakePower())
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

	public void periodic() {
		buttonDigitalInput.updateInputs(buttonInputs);
		Logger.processInputs(logPath + "/IsIntakeButtonHeld", buttonInputs);

		if (!hasFourBarBeenReset() && fourBar.getCurrent() > FourBarConstants.CURRENT_THRESHOLD_TO_RESET_POSITION) {
			hasFourBarBeenReset = true;
		}

		if (FourBarConstants.MAXIMUM_POSITION.getRadians() < fourBar.getPosition().getRadians() && !hasFourBarBeenReset()) {
			fourBar.setPosition(FourBarConstants.MAXIMUM_POSITION);
		}

		Logger.recordOutput(logPath + "/HasFourBarBeenReset", hasFourBarBeenReset());
	}

	public boolean hasFourBarBeenReset() {
		return hasFourBarBeenReset;
	}

	public IntakeState getCurrentState() {
		return currentState;
	}

}
