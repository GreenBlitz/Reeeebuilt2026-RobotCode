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
import java.util.function.Supplier;

public class IntakeStateHandler {

	private final Arm fourBar;
	private final Roller rollers;
	private final BooleanSupplier isReset;
	private final String logPath;
	private final LoggedNetworkNumber rollersCalibrationPower = new LoggedNetworkNumber("Tunable/IntakeRollerPower");
	private final LoggedNetworkRotation2d fourBarCalibrationPosition = new LoggedNetworkRotation2d("Tunable/FourBarPosition", new Rotation2d());

	private IntakeState currentState;

	public IntakeStateHandler(Arm fourBar, Roller rollers, BooleanSupplier isReset, String logPath) {
		this.fourBar = fourBar;
		this.rollers = rollers;
		this.isReset = isReset;
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
			case CLOSED -> close();
		},
			new InstantCommand(() -> Logger.recordOutput(logPath + "/CurrentState", intakeState.name())),
			new InstantCommand(() -> currentState = intakeState)
		);
	}

	public IntakeState getCurrentState() {
		return currentState;
	}

}
