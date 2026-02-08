package frc.robot.subsystems.constants.hood;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.IDs;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.digitalinput.channeled.ChanneledDigitalInput;
import frc.robot.hardware.digitalinput.chooser.ChooserDigitalInput;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmSimulationConstants;
import frc.robot.subsystems.arm.TalonFXArmBuilder;

public class HoodConstants {

	public static final FeedbackConfigs FEEDBACK_CONFIGS = new FeedbackConfigs();
	public static final Slot0Configs REAL_SLOT = new Slot0Configs();
	public static final Slot0Configs SIMULATION_SLOT = new Slot0Configs();

	static {
		REAL_SLOT.kP = 28;
		REAL_SLOT.kI = 0;
		REAL_SLOT.kD = 0;
		REAL_SLOT.kS = 0.065;
		REAL_SLOT.kG = 0.37;
		REAL_SLOT.kV = 9.0000095367432;
		REAL_SLOT.kA = 0.5209;
		REAL_SLOT.GravityType = GravityTypeValue.Arm_Cosine;

		SIMULATION_SLOT.kP = 150;
		SIMULATION_SLOT.kI = 0;
		SIMULATION_SLOT.kD = 0;
		SIMULATION_SLOT.kG = 0;
		SIMULATION_SLOT.kS = 0;
		SIMULATION_SLOT.GravityType = GravityTypeValue.Arm_Cosine;

		FEEDBACK_CONFIGS.RotorToSensorRatio = 1;
		FEEDBACK_CONFIGS.SensorToMechanismRatio = 450 / 7.0;
	}

	public static final boolean IS_INVERTED = true;
	public static final double ARBITRARY_FEEDFORWARD = 0;
	public static final double CURRENT_LIMIT = 40;
	public static final double MOMENT_OF_INERTIA = 0.001;
	public static final double HOOD_LENGTH_METERS = 0.3;
	public static final Rotation2d FORWARD_SOFTWARE_LIMIT = Rotation2d.fromDegrees(80);
	public static final Rotation2d BACKWARD_SOFTWARE_LIMIT = Rotation2d.fromDegrees(0);
	public static final Rotation2d MINIMUM_POSITION = Rotation2d.fromDegrees(BACKWARD_SOFTWARE_LIMIT.getDegrees() - 3);
	public static final Rotation2d MAXIMUM_POSITION = Rotation2d.fromDegrees(FORWARD_SOFTWARE_LIMIT.getDegrees() + 3);
	public static final Rotation2d DEFAULT_MAX_ACCELERATION_PER_SECOND_SQUARE = Rotation2d.fromRotations(3);
	public static final Rotation2d DEFAULT_MAX_VELOCITY_PER_SECOND = Rotation2d.fromRotations(3);
	public static final SysIdRoutine.Config SYSIDROUTINE_CONFIG = new SysIdRoutine.Config();
	public static final boolean IS_CONTINUOUS_WRAP = false;
	public static final boolean IS_RESET_CHECK_SENSOR_INVERTED = true;
	public static final double RESET_CHECK_SENSOR_DEBOUNCE_TIME = 0.15;
	public static final double RESET_HOOD_VOLTAGE = -0.7;

	public static Arm createHood() {
		ArmSimulationConstants hoodSimulationConstants = new ArmSimulationConstants(
			MAXIMUM_POSITION,
			MINIMUM_POSITION,
			MINIMUM_POSITION,
			MOMENT_OF_INERTIA,
			HOOD_LENGTH_METERS
		);
		return TalonFXArmBuilder.buildDynamicMotionMagicArm(
			RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Hood",
			IDs.TalonFXIDs.HOOD,
			IS_INVERTED,
			IS_CONTINUOUS_WRAP,
			new TalonFXFollowerConfig(),
			SYSIDROUTINE_CONFIG,
			FEEDBACK_CONFIGS,
			REAL_SLOT,
			SIMULATION_SLOT,
			CURRENT_LIMIT,
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			ARBITRARY_FEEDFORWARD,
			FORWARD_SOFTWARE_LIMIT,
			BACKWARD_SOFTWARE_LIMIT,
			hoodSimulationConstants,
			DEFAULT_MAX_ACCELERATION_PER_SECOND_SQUARE,
			DEFAULT_MAX_VELOCITY_PER_SECOND
		);
	}

	public static IDigitalInput createHoodResetCheckSensor() {
		return Robot.ROBOT_TYPE.isReal()
			? new ChanneledDigitalInput(
				new DigitalInput(IDs.DigitalInputsIDs.HOOD_RESET_SENSOR),
				new Debouncer(RESET_CHECK_SENSOR_DEBOUNCE_TIME),
				IS_RESET_CHECK_SENSOR_INVERTED
			)
			: new ChooserDigitalInput("hoodResetCheck");
	}

}

