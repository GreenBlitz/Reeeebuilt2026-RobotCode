package frc.robot.subsystems.constants.upperRoller;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.TalonFXRollerBuilder;

public class UpperRollerConstants {

	public static final String LOG_PATH = RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/UpperRoller";

	public final static boolean IS_INVERTED = true;
	public final static FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs();

	public final static int CURRENT_LIMIT = 60;
	public final static double MOMENT_OF_INERTIA = 0.0001;

	public static Roller createUpperRoller() {
		return TalonFXRollerBuilder
			.build(LOG_PATH, IDs.TalonFXIDs.UPPER_ROLLER, IS_INVERTED, FEEDBACK_CONFIGS, CURRENT_LIMIT, MOMENT_OF_INERTIA);
	}

}
