package frc.robot.statemachine.shooterstatehandler;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.constants.turret.TurretConstants;
import frc.utils.math.ToleranceMath;

public class TurretCalculations {

	public static Rotation2d getRangeEdge(Rotation2d angle, Rotation2d tolerance) {
		return getWrappedTurretPosition(Rotation2d.fromDegrees(angle.getDegrees() + tolerance.getDegrees()));
	}

	public static boolean isTurretMoveLegal(Rotation2d targetRobotRelative, Rotation2d position) {
		boolean isTargetInMaxRange = !(targetRobotRelative.getDegrees() > TurretConstants.SCREW_MAX_RANGE_EDGE.getDegrees()
			&& position.getDegrees() < TurretConstants.SCREW_MIN_RANGE_EDGE.getDegrees());

		boolean isTargetInMinRange = !(targetRobotRelative.getDegrees() < TurretConstants.SCREW_MIN_RANGE_EDGE.getDegrees()
			&& position.getDegrees() > TurretConstants.SCREW_MAX_RANGE_EDGE.getDegrees());

		boolean isTargetBehindSoftwareLimits = ToleranceMath.isInRange(
			targetRobotRelative.getDegrees(),
			TurretConstants.BACKWARDS_SOFTWARE_LIMIT.getDegrees(),
			TurretConstants.FORWARD_SOFTWARE_LIMIT.getDegrees()
		);

		return isTargetInMaxRange && isTargetInMinRange && isTargetBehindSoftwareLimits;
	}

	public static Rotation2d getWrappedTurretPosition(Rotation2d position) {
		return Rotation2d.fromRadians(
			MathUtil.inputModulus(position.getRadians(), TurretConstants.MIN_POSITION.getRadians(), TurretConstants.MAX_POSITION.getRadians())
		);
	}

}
