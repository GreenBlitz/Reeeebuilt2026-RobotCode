package frc.robot.subsystems.constants.train;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.IDs;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.digitalinput.channeled.ChanneledDigitalInput;
import frc.robot.hardware.digitalinput.chooser.ChooserDigitalInput;
import frc.robot.subsystems.roller.TalonFXRollerBuilder;
import frc.robot.subsystems.roller.VelocityRoller;

public class TrainConstant {

	public static final String LOG_PATH = RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Train";

	public static final boolean IS_INVERTED = false;

	public static final int CURRENT_LIMIT = 20;
	public static final double MOMENT_OF_INERTIA = 0.001;

	public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs();
	public static final Slot0Configs REAL_SLOTS_CONFIG = new Slot0Configs();
	public static final Slot0Configs SIMULATION_SLOTS_CONFIG = new Slot0Configs();

	public static final double TRAIN_BALL_SENSOR_DEBOUNCE_TIME = 0.15;
	public static final boolean SENSOR_INVERTED = true;

	static {
		REAL_SLOTS_CONFIG.kP = 2;
		REAL_SLOTS_CONFIG.kI = 0;
		REAL_SLOTS_CONFIG.kD = 0;
		REAL_SLOTS_CONFIG.kG = 0;
		REAL_SLOTS_CONFIG.kS = 0.23;
		REAL_SLOTS_CONFIG.kV = 0.47;
		REAL_SLOTS_CONFIG.kA = 0;

		SIMULATION_SLOTS_CONFIG.kP = 0;
		SIMULATION_SLOTS_CONFIG.kI = 0;
		SIMULATION_SLOTS_CONFIG.kD = 0;
		SIMULATION_SLOTS_CONFIG.kG = 0;
		SIMULATION_SLOTS_CONFIG.kS = 0;
		SIMULATION_SLOTS_CONFIG.kV = 0.52;
		SIMULATION_SLOTS_CONFIG.kA = 0;

		FEEDBACK_CONFIGS.RotorToSensorRatio = 1;
		FEEDBACK_CONFIGS.SensorToMechanismRatio = 25.0 / 6.0;
	}

	public static VelocityRoller createTrain() {
		return TalonFXRollerBuilder.buildVelocityRoller(
			LOG_PATH,
			IDs.TalonFXIDs.TRAIN,
			REAL_SLOTS_CONFIG,
			SIMULATION_SLOTS_CONFIG,
			CURRENT_LIMIT,
			FEEDBACK_CONFIGS,
			MOMENT_OF_INERTIA,
			IS_INVERTED
		);
	}

	public static IDigitalInput createTrainBallSensor() {
		return Robot.ROBOT_TYPE.isReal()
			? new ChanneledDigitalInput(
				new DigitalInput(IDs.DigitalInputsIDs.TRAIN_BALL_SENSOR),
				new Debouncer(TRAIN_BALL_SENSOR_DEBOUNCE_TIME),
				SENSOR_INVERTED
			)
			: new ChooserDigitalInput("trainBallSensor");
	}

}
