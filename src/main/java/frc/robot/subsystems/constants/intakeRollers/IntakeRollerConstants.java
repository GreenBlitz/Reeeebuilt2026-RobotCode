package frc.robot.subsystems.constants.intakeRollers;

import frc.robot.IDs;
import frc.robot.RobotConstants;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.SparkMaxRollerBuilder;

public class IntakeRollerConstants {

	public final static int GEAR_RATIO = 10;
	public final static int CURRENT_LIMIT = 20;
	public final static double MOMENT_OF_INERTIA = 0.0001;
	public final static boolean IS_INVERTED = false;

	public static Roller createIntakeRollers() {
		return SparkMaxRollerBuilder.build(
			RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/IntakeRollers",
			IDs.SparkMAXIDs.INTAKE_ROLLERS,
			IS_INVERTED,
			GEAR_RATIO,
			CURRENT_LIMIT,
			MOMENT_OF_INERTIA
		);
	}

}
