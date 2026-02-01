package frc.robot.statemachine.intakestatehandler;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.constants.fourBar.FourBarConstants;
import frc.robot.subsystems.roller.Roller;
import frc.utils.LoggedNetworkRotation2d;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;
import java.util.Set;
import java.util.function.BooleanSupplier;

public class IntakeStateHandler {

	private final Arm fourBar;
	private final Roller rollers;
	private BooleanSupplier hasBeenReset;
	private final IDigitalInput fourBarResetCheckSensor;
	private final DigitalInputInputsAutoLogged fourBarResetCheckInput;
	private final String logPath;
	private final LoggedNetworkNumber rollersCalibrationPower = new LoggedNetworkNumber("Tunable/IntakeRollerPower");
	private final LoggedNetworkRotation2d fourBarCalibrationPosition = new LoggedNetworkRotation2d("Tunable/FourBarPosition", new Rotation2d());

	private IntakeState currentState;

	public IntakeStateHandler(Arm fourBar, Roller rollers, IDigitalInput fourBarResetCheckSensor, BooleanSupplier hasBeenReset, String logPath) {
		this.fourBar = fourBar;
		this.rollers = rollers;
		this.hasBeenReset = hasBeenReset;
		this.fourBarResetCheckSensor = fourBarResetCheckSensor;
		this.fourBarResetCheckInput = new DigitalInputInputsAutoLogged();
		this.logPath = logPath + "/IntakeStateHandler";
		this.currentState = IntakeState.STAY_IN_PLACE;
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
		return !hasBeenReset.getAsBoolean()
			? fourBar.getCommandsBuilder().setVoltageWithoutLimit(FourBarConstants.FOUR_BAR_RESET_VOLTAGE).until(() -> isFourBarReset())
			: new InstantCommand();
	}

	public Command toggleState() {
		return new InstantCommand(
			() -> CommandScheduler.getInstance()
				.schedule(
					new DeferredCommand(
						() -> new ConditionalCommand(
							setState(IntakeState.INTAKE),
							setState(IntakeState.CLOSED),
							() -> currentState == IntakeState.CLOSED
						),
						Set.of(fourBar, rollers)
					)
				)
		);
	}

	public Command intake() {
		return new ParallelCommandGroup(
			fourBar.getCommandsBuilder().setTargetPosition(IntakeState.INTAKE.getFourBarPosition()),
			rollers.getCommandsBuilder().setPower(IntakeState.INTAKE.getIntakePower())
		);
	}

	public Command close() {
		return new ParallelCommandGroup(
			fourBar.getCommandsBuilder().setTargetPosition(IntakeState.CLOSED.getFourBarPosition()),
			rollers.getCommandsBuilder().setPower(IntakeState.CLOSED.getIntakePower())
		);
	}

	public Command setState(IntakeState intakeState) {
		return new ParallelCommandGroup(switch (intakeState) {
			case CALIBRATION -> calibration();
			case STAY_IN_PLACE -> stayInPlace();
			case INTAKE -> intake();
			case RESET_FOUR_BAR -> resetFourBar();
			case CLOSED -> close();
		},
			new InstantCommand(() -> Logger.recordOutput(logPath + "/CurrentState", intakeState.name())),
			new InstantCommand(() -> currentState = intakeState)
		);
	}

	public void periodic() {
		fourBarResetCheckSensor.updateInputs(fourBarResetCheckInput);
	}

	public boolean isFourBarReset() {
		return fourBarResetCheckInput.debouncedValue;
	}

	public IntakeState getCurrentState() {
		return currentState;
	}

}
