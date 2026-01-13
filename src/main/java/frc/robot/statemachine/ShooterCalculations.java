package frc.robot.statemachine;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.MathConstants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.constants.turret.TurretConstants;
import frc.utils.math.FieldMath;
import frc.utils.math.ToleranceMath;

public class ShooterCalculations {

	public static Pose2d getTurretPoseFiledRelative(Pose2d robotPose) {
		return new Pose2d(
			robotPose.getX() + robotPose.getRotation().getCos() * TurretConstants.TURRET_POSITION_RELATIVE_TO_ROBOT.getX() + robotPose.getRotation().getSin() * TurretConstants.TURRET_POSITION_RELATIVE_TO_ROBOT.getY(),
			robotPose.getY() + robotPose.getRotation().getCos() * TurretConstants.TURRET_POSITION_RELATIVE_TO_ROBOT.getY() + robotPose.getRotation().getSin() * TurretConstants.TURRET_POSITION_RELATIVE_TO_ROBOT.getX(),
			robotPose.getRotation()
		);
	}

	public static Rotation2d getRobotRelativeLookAtTowerAngleForTurret(Translation2d target, Pose2d fieldRelativeTurretPose) {
		Rotation2d targetAngle = Rotation2d
			.fromRadians(FieldMath.getRelativeTranslation(fieldRelativeTurretPose, target).getAngle().getRadians());
		return Rotation2d
			.fromDegrees(MathUtil.inputModulus(targetAngle.getDegrees(), Rotation2d.kZero.getDegrees(), MathConstants.FULL_CIRCLE.getDegrees()));
	}

	public static boolean isTurretMoveLegal(Rotation2d targetRobotRelative, Arm turret) {
		boolean isTargetInMaxRange = !(targetRobotRelative.getDegrees() > TurretConstants.SCREW_MAX_RANGE_EDGE.getDegrees()
			&& turret.getPosition().getDegrees() < TurretConstants.SCREW_MIN_RANGE_EDGE.getDegrees());

		boolean isTargetInMinRange = !(targetRobotRelative.getDegrees() < TurretConstants.SCREW_MIN_RANGE_EDGE.getDegrees()
			&& turret.getPosition().getDegrees() > TurretConstants.SCREW_MAX_RANGE_EDGE.getDegrees());

		boolean isTargetBehindSoftwareLimits = ToleranceMath.isInRange(
			targetRobotRelative.getDegrees(),
			TurretConstants.BACKWARDS_SOFTWARE_LIMIT.getDegrees(),
			TurretConstants.FORWARD_SOFTWARE_LIMIT.getDegrees()
		);

		return isTargetInMaxRange && isTargetInMinRange && isTargetBehindSoftwareLimits;
	}

}
