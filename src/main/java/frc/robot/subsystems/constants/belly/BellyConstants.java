package frc.robot.subsystems.constants.belly;

import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.SparkMaxRollerBuilder;

public class BellyConstants {

	public static final String LOG_PATH = RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Belly";
	public final static int GEAR_RATIO = 1 / 1;
	public final static int CURRENT_LIMIT = 40;
	public final static double MOMENT_OF_INERTIA = 0.0001;
	public final static boolean IS_INVERTED = true;

	public static Roller createBelly() {
		return SparkMaxRollerBuilder.build(LOG_PATH, IDs.SparkMAXIDs.BELLY, IS_INVERTED, GEAR_RATIO, CURRENT_LIMIT, MOMENT_OF_INERTIA);
	}

}
