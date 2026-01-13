package frc.robot.statemachine.shooterstatehandler;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.statemachine.ScoringHelpers;
import frc.robot.statemachine.ShooterCalculations;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.FlyWheel;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

public class ShooterStateHandler {

	private final Arm turret;
	private final Arm hood;
	private final FlyWheel flyWheel;
	private final Supplier<Pose2d> robotPose;
	private final String logPath;
	private ShooterState currentState;

	public ShooterStateHandler(Arm turret, Arm hood, FlyWheel flyWheel, Supplier<Pose2d> robotPose, String logPath) {
		this.turret = turret;
		this.hood = hood;
		this.flyWheel = flyWheel;
		this.robotPose = robotPose;
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
			turret.asSubsystemCommand(new TurretAimAtHubCommand(turret, robotPose, logPath), "Aim at hub"),
			hood.getCommandsBuilder()
				.setTargetPosition(
					() -> ShooterCalculations.hoodInterpolation(ScoringHelpers.getDistanceFromHub(robotPose.get().getTranslation()))
				),
			flyWheel.getCommandBuilder().setTargetVelocity(ShooterConstants.DEFAULT_FLYWHEEL_ROTATIONS_PER_SECOND)
		);
	}

	private Command shoot() {
		return new ParallelCommandGroup(
			turret.asSubsystemCommand(new TurretAimAtHubCommand(turret, robotPose, logPath), "Aim at hub"),
			hood.getCommandsBuilder()
				.setTargetPosition(
					() -> ShooterCalculations.hoodInterpolation(ScoringHelpers.getDistanceFromHub(robotPose.get().getTranslation()))
				),
			flyWheel.getCommandBuilder()
				.setVelocityAsSupplier(
					() -> ShooterCalculations.flywheelInterpolation(ScoringHelpers.getDistanceFromHub(robotPose.get().getTranslation()))
				)
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
