package frc.robot.statemachine.shooterstatehandler;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.constants.MathConstants;
import frc.robot.statemachine.ScoringHelpers;
import frc.robot.statemachine.ShooterCalculations;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.utils.math.FieldMath;
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

	public static Rotation2d hoodInterpolation(Double distanceFromTower) {
		return ShooterCalculations.HOOD_INTERPOLATION_MAP.get(distanceFromTower);
	}

	public static Rotation2d flywheelInterpolation(Double distanceFromTower) {
		return ShooterCalculations.FLYWHEEL_INTERPOLATION_MAP.get(distanceFromTower);
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
			aimAtHub(),
			hood.getCommandsBuilder()
				.setTargetPosition(() -> hoodInterpolation(ScoringHelpers.getDistanceFromHub(robotPose.get().getTranslation()))),
			flyWheel.getCommandBuilder().setTargetVelocity(ShooterConstants.DEFAULT_FLYWHEEL_ROTATIONS_PER_SECOND)
		);
	}

	private Command shoot() {
		return new ParallelCommandGroup(
			aimAtHub(),
			hood.getCommandsBuilder()
				.setTargetPosition(() -> hoodInterpolation(ScoringHelpers.getDistanceFromHub(robotPose.get().getTranslation()))),
			flyWheel.getCommandBuilder()
				.setVelocityAsSupplier(() -> flywheelInterpolation(ScoringHelpers.getDistanceFromHub(robotPose.get().getTranslation())))
		);
	}

	private Command calibration() {
		return new ParallelCommandGroup(
			turret.getCommandsBuilder().setTargetPosition(() -> ShooterConstants.turretCalibrationAngle.get()),
			hood.getCommandsBuilder().setTargetPosition(() -> ShooterConstants.hoodCalibrationAngle.get()),
			flyWheel.getCommandBuilder().setVelocityAsSupplier(() -> ShooterConstants.flywheelCalibrationRotations.get())
		);
	}

	public Command aimAtHub() {
		return new TurretAimAtHubCommand(turret, robotPose, logPath);
	}

	public static Rotation2d getRobotRelativeLookAtHubAngleForTurret(Translation2d target, Pose2d fieldRelativeTurretPose) {
		Rotation2d targetAngle = Rotation2d
			.fromRadians(FieldMath.getRelativeTranslation(fieldRelativeTurretPose, target).getAngle().getRadians());
		return Rotation2d
			.fromDegrees(MathUtil.inputModulus(targetAngle.getDegrees(), Rotation2d.kZero.getDegrees(), MathConstants.FULL_CIRCLE.getDegrees()));
	}

	public static Rotation2d getRangeEdge(Rotation2d angle, Rotation2d tolerance) {
		return Rotation2d.fromRadians(
			MathUtil
				.inputModulus(angle.getRadians() + tolerance.getRadians(), Rotation2d.kZero.getRadians(), MathConstants.FULL_CIRCLE.getRadians())
		);
	}

}
