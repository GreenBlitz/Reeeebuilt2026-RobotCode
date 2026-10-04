package frc.robot.subsystems.constants.intakeRollers;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.TalonFXRollerBuilder;

public class IntakeRollerConstants {

	public static final boolean IS_INVERTED = true;

	public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs();

	static {
		FEEDBACK_CONFIGS.SensorToMechanismRatio = 1;
		FEEDBACK_CONFIGS.RotorToSensorRatio = 1;
	}

	public static final int CURRENT_LIMIT = 60;

	public static final double MOMENT_OF_INERTIA = 0.0001;

	public static final double TIME_TO_INTAKE_WHILE_CLOSING_SECONDS = 0.5;

	public static Roller createIntakeRollers() {
		return TalonFXRollerBuilder.build(
			RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/IntakeRollers",
			IDs.TalonFXIDs.INTAKE_ROLLERS,
			IS_INVERTED,
			FEEDBACK_CONFIGS,
			CURRENT_LIMIT,
			MOMENT_OF_INERTIA
		);
	}

}
