package frc.robot.subsystems.constants.conveyor;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.TalonFXRollerBuilder;

public class ConveyorConstants {

	public static final String LOG_PATH = RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Conveyor";

	public final static boolean IS_INVERTED = true;
	public final static FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs();

	static {
		FEEDBACK_CONFIGS.SensorToMechanismRatio = 1;
		FEEDBACK_CONFIGS.RotorToSensorRatio = 1;
	}

	public final static int CURRENT_LIMIT = 60;
	public final static double MOMENT_OF_INERTIA = 0.0001;

	public static final double STUCK_CURRENT = 90;

	public static Roller createConveyor() {
		return TalonFXRollerBuilder.build(LOG_PATH, IDs.TalonFXIDs.CONVEYOR, IS_INVERTED, FEEDBACK_CONFIGS, CURRENT_LIMIT, MOMENT_OF_INERTIA);
	}

}
