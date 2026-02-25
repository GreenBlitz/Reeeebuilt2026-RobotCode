package frc.robot.subsystems.swerve.factories.modules.constants;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.factories.constants.RealSwerveConstants;
import frc.robot.subsystems.swerve.module.ModuleUtil;
import frc.robot.subsystems.swerve.module.records.ModuleSpecificConstants;

class RealModuleConstants {

	private static final double WHEEL_DIAMETER_METERS = 0.048359 * 2;
	private static final double FRONT_MODULES_COUPLING_RATIO = 0.438231;
	private static final double BACK_MODULES_COUPLING_RATIO = 0.438231;

	private static final double MODULE_X_DISTANCE_FROM_CENTER = 0.24683;
	private static final double MODULE_Y_DISTANCE_FROM_CENTER = 0.30833;
	private static final Translation2d FRONT_LEFT_TRANSLATION2D = new Translation2d(
		MODULE_X_DISTANCE_FROM_CENTER,
		MODULE_Y_DISTANCE_FROM_CENTER
	);
	private static final Translation2d FRONT_RIGHT_TRANSLATION2D = new Translation2d(
		MODULE_X_DISTANCE_FROM_CENTER,
		-MODULE_Y_DISTANCE_FROM_CENTER
	);
	private static final Translation2d BACK_LEFT_TRANSLATION2D = new Translation2d(
		-MODULE_X_DISTANCE_FROM_CENTER,
		MODULE_Y_DISTANCE_FROM_CENTER
	);
	private static final Translation2d BACK_RIGHT_TRANSLATION2D = new Translation2d(
		-MODULE_X_DISTANCE_FROM_CENTER,
		-MODULE_Y_DISTANCE_FROM_CENTER
	);
	private static final Translation2d[] LOCATIONS = {
		FRONT_LEFT_TRANSLATION2D,
		FRONT_RIGHT_TRANSLATION2D,
		BACK_LEFT_TRANSLATION2D,
		BACK_RIGHT_TRANSLATION2D,};

	static ModuleSpecificConstants getModuleSpecificConstants(String logPath, ModuleUtil.ModulePosition modulePosition) {
		double couplingRatio = switch (modulePosition) {
			case FRONT_LEFT, FRONT_RIGHT -> FRONT_MODULES_COUPLING_RATIO;
			case BACK_LEFT, BACK_RIGHT -> BACK_MODULES_COUPLING_RATIO;
		};
		return new ModuleSpecificConstants(
			modulePosition,
			logPath,
			WHEEL_DIAMETER_METERS,
			couplingRatio,
			RealSwerveConstants.VELOCITY_AT_12_VOLTS_METERS_PER_SECOND,
			LOCATIONS[modulePosition.getIndex()]
		);
	}

}
