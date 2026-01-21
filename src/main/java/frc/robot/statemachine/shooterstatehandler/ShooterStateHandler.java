package frc.robot.statemachine.shooterstatehandler;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.FlyWheel;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ShooterStateHandler {

	private final Arm turret;
	private final Arm hood;
	private final FlyWheel flyWheel;
	private final Supplier<ShootingParams> shootingParamsSupplier;
	private final String logPath;
	private ShooterState currentState;

	public ShooterStateHandler(Arm turret, Arm hood, FlyWheel flyWheel, Supplier<ShootingParams> shootingParamsSupplier, String logPath) {
		this.turret = turret;
		this.hood = hood;
		this.flyWheel = flyWheel;
		this.shootingParamsSupplier = shootingParamsSupplier;
		this.currentState = ShooterState.STAY_IN_PLACE;
		this.logPath = logPath + "/ShooterStateHandler";
	}

	public ShooterState getCurrentState() {
		return currentState;
	}

	public Command setState(ShooterState shooterState) {
		Command command = switch (shooterState) {
			case STAY_IN_PLACE -> stayInPlace();
			case IDLE -> idle();
			case SHOOT -> shoot();
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

	private Command idle() {
		return new ParallelCommandGroup(
			turret.asSubsystemCommand(
				new TurretSafeMoveToPosition(turret, () -> shootingParamsSupplier.get().targetTurretPosition(), logPath),
				"Safe move to position"
			),
			hood.getCommandsBuilder().setTargetPosition(() -> shootingParamsSupplier.get().targetHoodPosition()),
			flyWheel.getCommandBuilder().setTargetVelocity(ShooterConstants.DEFAULT_FLYWHEEL_ROTATIONS_PER_SECOND)
		);
	}

	private Command shoot() {
		return new ParallelCommandGroup(
			turret.asSubsystemCommand(
				new TurretSafeMoveToPosition(turret, () -> shootingParamsSupplier.get().targetTurretPosition(), logPath),
				"Safe move to position"
			),
			hood.getCommandsBuilder().setTargetPosition(() -> shootingParamsSupplier.get().targetHoodPosition()),
			flyWheel.getCommandBuilder().setVelocityAsSupplier(() -> shootingParamsSupplier.get().targetFlywheelVelocityRPS())
		);
	}

	private Command calibration() {
		return new ParallelCommandGroup(
			turret.getCommandsBuilder().setTargetPosition(() -> ShooterConstants.turretCalibrationAngle.get()),
			hood.getCommandsBuilder().setTargetPosition(() -> ShooterConstants.hoodCalibrationAngle.get()),
			flyWheel.getCommandBuilder().setVelocityAsSupplier(() -> ShooterConstants.flywheelCalibrationRotations.get())
		);
	}

}
