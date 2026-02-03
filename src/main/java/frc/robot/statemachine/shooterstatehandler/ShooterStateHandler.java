package frc.robot.statemachine.shooterstatehandler;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.hardware.digitalinput.DigitalInputInputsAutoLogged;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.statemachine.ShootingCalculations;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.VelocityPositionArm;
import frc.robot.subsystems.constants.hood.HoodConstants;
import frc.robot.subsystems.constants.turret.TurretConstants;
import frc.robot.subsystems.flywheel.FlyWheel;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ShooterStateHandler {

	private final VelocityPositionArm turret;
	private final Arm hood;
	private final FlyWheel flyWheel;
	private final Supplier<ShootingParams> shootingParamsSupplier;
	private final IDigitalInput turretResetCheckSensor;
	private final IDigitalInput hoodResetCheckSensor;
	private final DigitalInputInputsAutoLogged turretResetCheckInput;
	private final DigitalInputInputsAutoLogged hoodResetCheckInput;
	private final String logPath;
	private boolean hasHoodBeenReset;
	private boolean hasTurretBeenReset;
	private ShooterState currentState;

	public ShooterStateHandler(
		VelocityPositionArm turret,
		Arm hood,
		FlyWheel flyWheel,
		Supplier<ShootingParams> shootingParamsSupplier,
		IDigitalInput turretResetCheckSensor,
		IDigitalInput hoodResetCheckSensor,
		String logPath
	) {
		this.turret = turret;
		this.hood = hood;
		this.flyWheel = flyWheel;
		this.shootingParamsSupplier = shootingParamsSupplier;
		this.turretResetCheckSensor = turretResetCheckSensor;
		this.hoodResetCheckSensor = hoodResetCheckSensor;
		this.turretResetCheckInput = new DigitalInputInputsAutoLogged();
		this.hoodResetCheckInput = new DigitalInputInputsAutoLogged();
		this.currentState = ShooterState.STAY_IN_PLACE;
		this.hasHoodBeenReset = isHoodAtSensor();
		this.hasTurretBeenReset = isTurretAtSensor();
		this.logPath = logPath + "/ShooterStateHandler";
	}

	public ShooterState getCurrentState() {
		return currentState;
	}

	public Command setState(ShooterState shooterState) {
		Command command = switch (shooterState) {
			case STAY_IN_PLACE -> stayInPlace();
			case NEUTRAL -> neutral();
			case SHOOT -> shoot();
			case RESET_SUBSYSTEMS -> resetSubsystems();
			case CALIBRATION -> calibration();
		};
		return new ParallelCommandGroup(
			new InstantCommand(() -> Logger.recordOutput(logPath + "/CurrentState", shooterState.name())),
			new InstantCommand(() -> currentState = shooterState),
			command
		);
	}

	private Command stayInPlace() {
		return new ParallelCommandGroup(
			turret.getCommandsBuilder().stayInPlace(),
			hood.getCommandsBuilder().stayInPlace(),
			flyWheel.getCommandBuilder().stop()
		);
	}

	private Command neutral() {
		return new ParallelCommandGroup(
			turret.asSubsystemCommand(
				new TurretSafeMoveToPosition(
					turret,
					() -> shootingParamsSupplier.get().targetTurretPosition(),
					() -> ShootingCalculations.getShootingParams().targetTurretVelocityRPS(),
					logPath
				),
				"Safe move to position"
			),
			hood.getCommandsBuilder().setTargetPosition(() -> shootingParamsSupplier.get().targetHoodPosition()),
			flyWheel.getCommandBuilder().setTargetVelocity(ShooterConstants.DEFAULT_FLYWHEEL_ROTATIONS_PER_SECOND)
		);
	}

	private Command shoot() {
		return new ParallelCommandGroup(
			turret.asSubsystemCommand(
				new TurretSafeMoveToPosition(
					turret,
					() -> shootingParamsSupplier.get().targetTurretPosition(),
					() -> shootingParamsSupplier.get().targetTurretVelocityRPS(),
					logPath
				),
				"Safe move to position"
			),
			hood.getCommandsBuilder().setTargetPosition(() -> shootingParamsSupplier.get().targetHoodPosition()),
			flyWheel.getCommandBuilder().setVelocityAsSupplier(() -> shootingParamsSupplier.get().targetFlywheelVelocityRPS())
		);
	}

	private Command resetSubsystems() {
		return new ParallelCommandGroup(
			hood.getCommandsBuilder().setVoltageWithoutLimit(HoodConstants.RESET_HOOD_VOLTAGE, () -> hasHoodBeenReset()),
			turret.getCommandsBuilder().setVoltageWithoutLimit(TurretConstants.RESET_TURRET_VOLTAGE, () -> hasTurretBeenReset())
		);
	}

	private Command calibration() {
		return new ParallelCommandGroup(
			turret.getCommandsBuilder().setTargetPosition(() -> ShooterConstants.turretCalibrationAngle.get()),
			hood.getCommandsBuilder().setTargetPosition(() -> ShooterConstants.hoodCalibrationAngle.get()),
			flyWheel.getCommandBuilder().setVelocityAsSupplier(() -> ShooterConstants.flywheelCalibrationRotations.get())
		);
	}

	public boolean isTurretAtSensor() {
		return turretResetCheckInput.debouncedValue;
	}

	public boolean isHoodAtSensor() {
		return hoodResetCheckInput.debouncedValue;
	}

	public void periodic() {
		turretResetCheckSensor.updateInputs(turretResetCheckInput);
		hoodResetCheckSensor.updateInputs(hoodResetCheckInput);
		if (!hasTurretBeenReset)
			hasTurretBeenReset = isTurretAtSensor();
		if (!hasHoodBeenReset)
			hasHoodBeenReset = isHoodAtSensor();
		Logger.recordOutput(logPath + "/HasHoodBeenReset", hasHoodBeenReset);
		Logger.recordOutput(logPath + "/HasTurretBeenReset", hasTurretBeenReset);
		Logger.processInputs(logPath + "/turretResetSensor", turretResetCheckInput);
		Logger.processInputs(logPath + "/hoodResetSensor", hoodResetCheckInput);
		Logger.recordOutput(logPath + "/CurrentState", currentState);
	}

	public boolean hasTurretBeenReset() {
		return hasTurretBeenReset;
	}

	public boolean hasHoodBeenReset() {
		return hasHoodBeenReset;
	}

	public boolean hasBeenFullyReset() {
		return hasTurretBeenReset && hasHoodBeenReset;
	}

}
