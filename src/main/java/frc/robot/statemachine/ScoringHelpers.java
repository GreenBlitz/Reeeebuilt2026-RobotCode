package frc.robot.statemachine;

import edu.wpi.first.math.geometry.Translation2d;
import frc.constants.field.Field;

public class ScoringHelpers {

	public static double getDistanceFromHub(Translation2d pose) {
		return Field.getHubMiddle().getDistance(pose);
	}

}
