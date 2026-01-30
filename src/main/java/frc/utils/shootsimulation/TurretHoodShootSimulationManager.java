package frc.utils.shootsimulation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.*;

import java.util.function.Supplier;

public class TurretHoodShootSimulationManager {

	private final Supplier<Pose2d> robotPose;
	private final Supplier<ChassisSpeeds> fieldRelativeSpeeds;
	private final Supplier<Rotation2d> turretPosition;
	private final Supplier<Rotation2d> hoodPosition;
	private final Supplier<Rotation2d> flywheelVelocity;
	private final Translation3d turretOffset;
	private final double wheelRadius;
	private final double stopHeight;

	private int amount = 0;

	public TurretHoodShootSimulationManager(
		Supplier<Pose2d> robotPose,
		Supplier<ChassisSpeeds> fieldRelativeSpeeds,
		Supplier<Rotation2d> turretPosition,
		Supplier<Rotation2d> hoodPosition,
		Supplier<Rotation2d> flywheelVelocity,
		Translation3d turretOffset,
		double wheelRadius,
		double stopHeight
	) {
		this.robotPose = robotPose;
		this.fieldRelativeSpeeds = fieldRelativeSpeeds;
		this.turretPosition = turretPosition;
		this.hoodPosition = hoodPosition;
		this.flywheelVelocity = flywheelVelocity;
		this.turretOffset = turretOffset;
		this.wheelRadius = wheelRadius;
		this.stopHeight = stopHeight;
	}

	public void run(String logPath) {
		Pose2d robotPose = this.robotPose.get();
		ChassisSpeeds fieldRelativeSpeeds = this.fieldRelativeSpeeds.get();
		Rotation2d turretPosition = this.turretPosition.get();
		Rotation2d hoodPosition = this.hoodPosition.get();
		double launchSpeedMetersPerSecond = 0.21 * flywheelVelocity.get().getRadians() * wheelRadius;

		Translation2d turretPos2d = turretOffset.toTranslation2d().rotateBy(robotPose.getRotation());
		Translation3d launchPosition = new Translation3d(
			robotPose.getX() + turretPos2d.getX(),
			robotPose.getY() + turretPos2d.getY(),
			turretOffset.getZ()
		);

		Translation2d robotTranslationalVel = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);
		Translation2d turretTangentialVel = turretOffset.toTranslation2d()
			.rotateBy(Rotation2d.kCCW_90deg)
			.times(fieldRelativeSpeeds.omegaRadiansPerSecond)
			.rotateBy(robotPose.getRotation());
		Translation2d totalTurretVel = robotTranslationalVel.plus(turretTangentialVel);

		Rotation2d fieldRelativeShootingAngle = robotPose.getRotation().plus(turretPosition);

		double launchX = launchSpeedMetersPerSecond * hoodPosition.getSin() * fieldRelativeShootingAngle.getCos();
		double launchY = launchSpeedMetersPerSecond * hoodPosition.getSin() * fieldRelativeShootingAngle.getSin();
		double launchZ = launchSpeedMetersPerSecond * hoodPosition.getCos();

		Translation3d totalVelocity = new Translation3d(totalTurretVel.getX() + launchX, totalTurretVel.getY() + launchY, launchZ);

		CommandScheduler.getInstance().schedule(new ShootSimulation(logPath, launchPosition, totalVelocity, stopHeight));
	}

	public void run(int amount, double timeInBetween) {
		this.amount = amount;
		CommandScheduler.getInstance()
			.schedule(
				new RepeatCommand(
					new SequentialCommandGroup(
						new InstantCommand(() -> run(this.amount + "")),
						new InstantCommand(() -> this.amount--),
						new WaitCommand(timeInBetween)
					)
				).until(() -> this.amount == 0)
			);
	}

}
