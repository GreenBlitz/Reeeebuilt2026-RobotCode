package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.subsystems.swerve.states.DriveSpeed;
import frc.utils.math.ToleranceMath;
import frc.utils.time.TimeUtil;

public class SwerveMath {

	public static double calculateDriveRadiusMeters(Translation2d[] modulePositionsFromCenterMeters) {
		double sum = 0;
		for (Translation2d modulePositionFromCenterMeters : modulePositionsFromCenterMeters) {
			sum += modulePositionFromCenterMeters.getDistance(new Translation2d());
		}
		return sum / modulePositionsFromCenterMeters.length;
	}

	public static ChassisSpeeds allianceToRobotRelativeSpeeds(ChassisSpeeds allianceRelativeSpeeds, Rotation2d allianceRelativeHeading) {
		return ChassisSpeeds.fromFieldRelativeSpeeds(allianceRelativeSpeeds, allianceRelativeHeading);
	}

	public static ChassisSpeeds robotToAllianceRelativeSpeeds(ChassisSpeeds robotRelativeSpeeds, Rotation2d allianceRelativeHeading) {
		return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, allianceRelativeHeading);
	}

	public static ChassisSpeeds discretize(ChassisSpeeds chassisSpeeds) {
		return ChassisSpeeds.discretize(chassisSpeeds, TimeUtil.getLatestCycleTimeSeconds());
	}

	public static ChassisSpeeds powersToSpeeds(ChassisPowers powers, SwerveConstants constants) {
		return new ChassisSpeeds(
			powers.xPower * constants.velocityAt12VoltsMetersPerSecond(),
			powers.yPower * constants.velocityAt12VoltsMetersPerSecond(),
			powers.rotationalPower * constants.maxRotationalVelocityPerSecond().getRadians()
		);
	}

	public static ChassisSpeeds factorSpeeds(ChassisSpeeds speeds, DriveSpeed driveSpeed) {
		return new ChassisSpeeds(
			speeds.vxMetersPerSecond * driveSpeed.getTranslationSpeedFactor(),
			speeds.vyMetersPerSecond * driveSpeed.getTranslationSpeedFactor(),
			speeds.omegaRadiansPerSecond * driveSpeed.getRotationSpeedFactor()
		);
	}

	public static ChassisSpeeds applyDeadband(ChassisSpeeds chassisSpeeds, Pose2d deadbands) {
		double xVelocityMetersPerSecond = ToleranceMath.applyDeadband(chassisSpeeds.vxMetersPerSecond, deadbands.getX());
		double yVelocityMetersPerSecond = ToleranceMath.applyDeadband(chassisSpeeds.vyMetersPerSecond, deadbands.getY());
		double rotationalVelocityRadiansPerSecond = ToleranceMath
			.applyDeadband(chassisSpeeds.omegaRadiansPerSecond, deadbands.getRotation().getRadians());

		return new ChassisSpeeds(xVelocityMetersPerSecond, yVelocityMetersPerSecond, rotationalVelocityRadiansPerSecond);
	}

	public static boolean isStill(ChassisSpeeds chassisSpeeds, Pose2d deadbands) {
		return Math.abs(chassisSpeeds.vxMetersPerSecond) <= deadbands.getX()
			&& Math.abs(chassisSpeeds.vyMetersPerSecond) <= deadbands.getY()
			&& Math.abs(chassisSpeeds.omegaRadiansPerSecond) <= deadbands.getRotation().getRadians();
	}

	public static double getDriveMagnitude(ChassisSpeeds chassisSpeeds) {
		return Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2) + Math.pow(chassisSpeeds.vyMetersPerSecond, 2));
	}

	public static boolean getIsSkidding(
		SwerveDriveKinematics kinematics,
		SwerveModuleState[] moduleStates,
		double skidRobotToModuleVelocityToleranceMetersPerSecond
	) {
		ChassisSpeeds swerveVelocity = kinematics.toChassisSpeeds(moduleStates);
		Translation2d swerveTranslationalVelocityMetersPerSecond = new Translation2d(
			swerveVelocity.vxMetersPerSecond,
			swerveVelocity.vyMetersPerSecond
		);

		SwerveModuleState[] moduleRotationalStates = kinematics
			.toSwerveModuleStates(new ChassisSpeeds(0, 0, swerveVelocity.omegaRadiansPerSecond));
		SwerveModuleState[] moduleTranslationalStates = getModuleTranslationalStates(moduleStates, moduleRotationalStates);

		for (SwerveModuleState moduleTranslationalState : moduleTranslationalStates) {
			if (
				!ToleranceMath.isNear(
					swerveTranslationalVelocityMetersPerSecond,
					new Translation2d(moduleTranslationalState.speedMetersPerSecond, moduleTranslationalState.angle),
					skidRobotToModuleVelocityToleranceMetersPerSecond
				)
			) {
				return true;
			}
		}
		return false;
	}

	public static SwerveModuleState[] getModuleTranslationalStates(
		SwerveModuleState[] moduleStates,
		SwerveModuleState[] moduleRotationalStates
	) {
		SwerveModuleState[] moduleTranslationalStates = new SwerveModuleState[Math.min(moduleStates.length, moduleRotationalStates.length)];
		for (int i = 0; i < moduleTranslationalStates.length; i++) {
			moduleTranslationalStates[i] = getModuleTranslationalState(moduleStates[i], moduleRotationalStates[i]);
		}
		return moduleTranslationalStates;
	}

	private static SwerveModuleState getModuleTranslationalState(SwerveModuleState moduleState, SwerveModuleState moduleRotationalState) {
		Translation2d moduleTranslationalVelocity = new Translation2d(moduleState.speedMetersPerSecond, moduleState.angle)
			.minus(new Translation2d(moduleRotationalState.speedMetersPerSecond, moduleRotationalState.angle));
		return new SwerveModuleState(moduleTranslationalVelocity.getNorm(), moduleTranslationalVelocity.getAngle());
	}

}
