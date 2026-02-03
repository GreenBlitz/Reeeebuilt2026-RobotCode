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

public class IntakeStateHandler {

	private final Arm fourBar;
	private final Roller rollers;
	private boolean hasBeenReset;
	private final IDigitalInput fourBarResetCheckSensor;
	private final DigitalInputInputsAutoLogged fourBarResetCheckInput;
	private final String logPath;
	private final LoggedNetworkNumber rollersCalibrationPower = new LoggedNetworkNumber("Tunable/IntakeRollerPower");
	private final LoggedNetworkRotation2d fourBarCalibrationPosition = new LoggedNetworkRotation2d("Tunable/FourBarPosition", new Rotation2d());

	private IntakeState currentState;

	public IntakeStateHandler(Arm fourBar, Roller rollers, IDigitalInput fourBarResetCheckSensor, String logPath) {
		this.fourBar = fourBar;
		this.rollers = rollers;
		this.fourBarResetCheckSensor = fourBarResetCheckSensor;
		this.fourBarResetCheckInput = new DigitalInputInputsAutoLogged();
		this.hasBeenReset = isFourBarAtSensor();
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
		if (!hasFourBarBeenReset())
			hasBeenReset = isFourBarAtSensor();
		Logger.recordOutput(logPath + "/hasBeenReset", hasFourBarBeenReset());
		Logger.processInputs(logPath + "/resetSensorValue", fourBarResetCheckInput);
	}

	public boolean isFourBarAtSensor() {
		return fourBarResetCheckInput.debouncedValue;
	}

	public boolean hasFourBarBeenReset() {
		return hasBeenReset;
	}

	public IntakeState getCurrentState() {
		return currentState;
	}

}
