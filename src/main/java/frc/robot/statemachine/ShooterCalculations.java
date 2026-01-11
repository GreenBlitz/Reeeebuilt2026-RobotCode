package frc.robot.statemachine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.MathConstants;
import frc.robot.subsystems.constants.turret.TurretConstants;
import frc.utils.math.FieldMath;

public class ShooterCalculations {

	public static Pose2d getTurretPose(Pose2d robotPose) {
		return new Pose2d(
			robotPose.getX() + robotPose.getRotation().getCos() * TurretConstants.TURRET_DISTANCE_FROM_ROBOT.getX(),
			robotPose.getY() + robotPose.getRotation().getSin() * TurretConstants.TURRET_DISTANCE_FROM_ROBOT.getX(),
			robotPose.getRotation()
		);
	}

	public static Rotation2d getRobotRelativeLookAtTowerAngleForTurret(Translation2d target, Pose2d fieldRelativeTurretPose) {
		Rotation2d targetAngle = Rotation2d
			.fromRadians(FieldMath.getRelativeTranslation(fieldRelativeTurretPose, target).getAngle().getRadians());
		return Rotation2d
			.fromDegrees(MathUtil.inputModulus(targetAngle.getDegrees(), Rotation2d.kZero.getDegrees(), MathConstants.FULL_CIRCLE.getDegrees()));
	}

}
