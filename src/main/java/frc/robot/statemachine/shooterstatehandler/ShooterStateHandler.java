package frc.robot.statemachine.shooterstatehandler;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.constants.MathConstants;
import frc.robot.statemachine.ScoringHelpers;
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

	public static Supplier<Rotation2d> hoodInterpolation(Supplier<Double> distanceFromTower) {
		return () -> ShooterConstants.HOOD_INTERPOLATION_MAP.get(distanceFromTower.get());
	}

	public static Supplier<Rotation2d> flywheelInterpolation(Supplier<Double> distanceFromTower) {
		return () -> ShooterConstants.FLYWHEEL_INTERPOLATION_MAP.get(distanceFromTower.get());
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
			aimAtTower(),
			hood.getCommandsBuilder().setTargetPosition(hoodInterpolation(() -> ScoringHelpers.getDistanceFromClosestTower(robotPose.get()))),
			flyWheel.getCommandBuilder().setTargetVelocity(ShooterConstants.DEFAULT_FLYWHEEL_ROTATIONS_PER_SECOND)
		);
	}

	private Command shoot() {
		return new ParallelCommandGroup(
			aimAtTower(),
			hood.getCommandsBuilder().setTargetPosition(hoodInterpolation(() -> ScoringHelpers.getDistanceFromClosestTower(robotPose.get()))),
			flyWheel.getCommandBuilder()
				.setVelocityAsSupplier(flywheelInterpolation(() -> ScoringHelpers.getDistanceFromClosestTower(robotPose.get())))
		);
	}

	private Command calibration() {
		return new ParallelCommandGroup(
			turret.getCommandsBuilder().setTargetPosition(() -> ShooterConstants.turretCalibrationAngle.get()),
			hood.getCommandsBuilder().setTargetPosition(() -> ShooterConstants.hoodCalibrationAngle.get()),
			flyWheel.getCommandBuilder().setVelocityAsSupplier(() -> ShooterConstants.flywheelCalibrationRotations.get())
		);
	}

	public Command aimAtTower() {
		return new TurretAimAtTowerCommand(turret, robotPose, logPath);
	}

	public static Rotation2d getRangeEdge(Rotation2d angle, Rotation2d tolerance) {
		return Rotation2d.fromRadians(
			MathUtil
				.inputModulus(angle.getRadians() + tolerance.getRadians(), Rotation2d.kZero.getRadians(), MathConstants.FULL_CIRCLE.getRadians())
		);
	}

}
