package frc.utils.pose;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.utils.math.ToleranceMath;

public class swerveUtil {

	public static Translation2d getMajority(Translation2d[] arr) {
		return ToleranceMath.isNear(arr[0], arr[1], SwerveConstants.SKID_TOLERANCE_VELOCITY_METERS_PER_SECOND)
			|| ToleranceMath.isNear(arr[0], arr[2], SwerveConstants.SKID_TOLERANCE_VELOCITY_METERS_PER_SECOND) ? arr[0] : arr[1];
	}

}
