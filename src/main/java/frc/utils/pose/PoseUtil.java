package frc.utils.pose;

import edu.wpi.first.math.geometry.Rotation2d;
<<<<<<< HEAD
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
=======
>>>>>>> template/master
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
<<<<<<< HEAD
import frc.constants.MathConstants;
import frc.robot.poseestimator.Pose2dComponentsValue;
import frc.robot.poseestimator.Pose3dComponentsValue;
import frc.utils.AngleUnit;
import frc.utils.alerts.Alert;
=======
>>>>>>> template/master
import frc.utils.math.ToleranceMath;

public class PoseUtil {

	public static boolean isAccelerationHigh(Translation2d imuAccelerationG, double minimumIMUAccelerationG) {
		return imuAccelerationG.getNorm() >= minimumIMUAccelerationG;
	}

	public static boolean isTilted(Rotation2d imuRoll, Rotation2d imuPitch, Rotation2d minimumTiltIMURoll, Rotation2d minimumTiltIMUPitch) {
		return Math.abs(imuRoll.getRadians()) >= minimumTiltIMURoll.getRadians()
			|| Math.abs(imuPitch.getRadians()) >= minimumTiltIMUPitch.getRadians();
	}

	public static boolean areModulesSkidding(
		SwerveDriveKinematics kinematics,
		SwerveModuleState[] moduleStates,
		double minimumSkidRobotToModuleVelocityDifferenceMetersPerSecond,
		double maximumNegligibleVectorNorm
	) {
		ChassisSpeeds swerveVelocity = kinematics.toChassisSpeeds(moduleStates);
		Translation2d swerveTranslationalVelocityMetersPerSecond = new Translation2d(
			swerveVelocity.vxMetersPerSecond,
			swerveVelocity.vyMetersPerSecond
		);

		SwerveModuleState[] moduleRotationalStates = kinematics
			.toSwerveModuleStates(new ChassisSpeeds(0, 0, swerveVelocity.omegaRadiansPerSecond));
		SwerveModuleState[] moduleTranslationalStates = getModuleTranslationalStates(
			moduleStates,
			moduleRotationalStates,
			maximumNegligibleVectorNorm
		);

		for (SwerveModuleState moduleTranslationalState : moduleTranslationalStates) {
			if (
				!ToleranceMath.isNear(
					swerveTranslationalVelocityMetersPerSecond,
					new Translation2d(moduleTranslationalState.speedMetersPerSecond, moduleTranslationalState.angle),
					minimumSkidRobotToModuleVelocityDifferenceMetersPerSecond
				)
			) {
				return true;
			}
		}
		return false;
	}

	private static SwerveModuleState[] getModuleTranslationalStates(
		SwerveModuleState[] moduleStates,
		SwerveModuleState[] moduleRotationalStates,
		double maximumNegligibleVectorNorm
	) {
		SwerveModuleState[] moduleTranslationalStates = new SwerveModuleState[Math.min(moduleStates.length, moduleRotationalStates.length)];
		for (int i = 0; i < moduleTranslationalStates.length; i++) {
			moduleTranslationalStates[i] = getModuleTranslationalState(moduleStates[i], moduleRotationalStates[i], maximumNegligibleVectorNorm);
		}
		return moduleTranslationalStates;
	}

	private static SwerveModuleState getModuleTranslationalState(
		SwerveModuleState moduleState,
		SwerveModuleState moduleRotationalState,
		double maximumNegligibleVectorNorm
	) {
		Translation2d velocityDifference = new Translation2d(moduleState.speedMetersPerSecond, moduleState.angle)
			.minus(new Translation2d(moduleRotationalState.speedMetersPerSecond, moduleRotationalState.angle));
		return velocityDifference.getNorm() > maximumNegligibleVectorNorm
			? new SwerveModuleState(velocityDifference.getNorm(), velocityDifference.getAngle())
			: new SwerveModuleState();
	}

	public static boolean getIsColliding(Translation2d imuAccelerationG, double minimumCollisionIMUAccelerationG) {
		return imuAccelerationG.getNorm() >= minimumCollisionIMUAccelerationG;
	}

	public static boolean getIsTilted(Rotation2d imuRoll, Rotation2d imuPitch, Rotation2d minimumTiltIMURoll, Rotation2d minimumTiltIMUPitch) {
		return Math.abs(imuRoll.getRadians()) >= minimumTiltIMURoll.getRadians()
			|| Math.abs(imuPitch.getRadians()) >= minimumTiltIMUPitch.getRadians();
	}

	public static boolean getIsSkidding(
		SwerveDriveKinematics kinematics,
		SwerveModuleState[] moduleStates,
		double minimumSkidRobotToModuleVelocityDifferenceMetersPerSecond
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
					minimumSkidRobotToModuleVelocityDifferenceMetersPerSecond
				)
			) {
				return true;
			}
		}
		return false;
	}

	private static SwerveModuleState[] getModuleTranslationalStates(
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
		return moduleTranslationalVelocity.getNorm() > MathConstants.MAXIMUM_NEGLIGIBLE_VECTOR_NORM
			? new SwerveModuleState(moduleTranslationalVelocity.getNorm(), moduleTranslationalVelocity.getAngle())
			: new SwerveModuleState();
	}

}
