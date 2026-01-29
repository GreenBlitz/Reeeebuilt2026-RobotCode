package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.util.function.Supplier;

public class TurretHoodShootSimulation {

	private final Supplier<Pose2d> robotPose;
	private final Supplier<ChassisSpeeds> robotVel;
	private final Supplier<Translation3d> turretOffset;
	private final Supplier<Rotation2d> turretPosition;
	private final Supplier<Rotation2d> hoodPosition;
	private final Supplier<Rotation2d> flywheelVelocity;
	private final double wheelRadius;
	private final double stopHeight;

	public TurretHoodShootSimulation(
		Supplier<Pose2d> robotPose,
		Supplier<ChassisSpeeds> robotVel,
		Supplier<Translation3d> turretOffset,
		Supplier<Rotation2d> turretPosition,
		Supplier<Rotation2d> hoodPosition,
		Supplier<Rotation2d> flywheelVelocity,
		double wheelRadius,
		double stopHeight
	) {
		this.robotPose = robotPose;
		this.robotVel = robotVel;
		this.turretOffset = turretOffset;
		this.turretPosition = turretPosition;
		this.hoodPosition = hoodPosition;
		this.flywheelVelocity = flywheelVelocity;
		this.wheelRadius = wheelRadius;
		this.stopHeight = stopHeight;
	}

	public void run() {
		Pose2d pose = robotPose.get();
		ChassisSpeeds speeds = robotVel.get();
		Rotation2d turretRot = turretPosition.get();
		Rotation2d hoodRot = hoodPosition.get();
		double launchSpeed = flywheelVelocity.get().getRadians() * wheelRadius;

		Translation2d turretPos2d = turretOffset.get().toTranslation2d().rotateBy(pose.getRotation());
		Translation3d launchPosition = new Translation3d(
			pose.getX() + turretPos2d.getX(),
			pose.getY() + turretPos2d.getY(),
			turretOffset.get().getZ()
		);

		Translation2d robotTranslationalVel = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
		Translation2d turretTangentialVel = turretOffset.get()
			.toTranslation2d()
			.rotateBy(Rotation2d.kCCW_90deg)
			.times(speeds.omegaRadiansPerSecond)
			.rotateBy(pose.getRotation());
		Translation2d inheritedVel = robotTranslationalVel.plus(turretTangentialVel);

		Rotation2d fieldRelativeShootingAngle = pose.getRotation().plus(turretRot);

		double launchX = launchSpeed * hoodRot.getCos() * fieldRelativeShootingAngle.getCos();
		double launchY = launchSpeed * hoodRot.getCos() * fieldRelativeShootingAngle.getSin();
		double launchZ = launchSpeed * hoodRot.getSin();

		Translation3d totalVelocity = new Translation3d(inheritedVel.getX() + launchX, inheritedVel.getY() + launchY, launchZ);

		CommandScheduler.getInstance().schedule(new ShootSimulation("ShotTrajectory", launchPosition, totalVelocity, stopHeight));
	}

}
