package frc.robot;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.hardware.digitalinput.IDigitalInput;
import frc.robot.hardware.digitalinput.channeled.ChanneledDigitalInput;
import frc.robot.hardware.digitalinput.chooser.ChooserDigitalInput;
import frc.robot.hardware.phoenix6.motors.TalonFXFollowerConfig;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmSimulationConstants;
import frc.robot.subsystems.arm.TalonFXArmBuilder;
import frc.robot.subsystems.arm.VelocityPositionArm;
import frc.robot.subsystems.constants.belly.BellyConstants;
import frc.robot.subsystems.constants.fourBar.FourBarConstants;
import frc.robot.subsystems.constants.hood.HoodConstants;
import frc.robot.subsystems.constants.intakeRollers.IntakeRollerConstants;
import frc.robot.subsystems.constants.train.TrainConstant;
import frc.robot.subsystems.constants.turret.TurretConstants;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.robot.subsystems.flywheel.KrakenX60FlyWheelBuilder;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.SparkMaxRollerBuilder;
import frc.robot.subsystems.roller.TalonFXRollerBuilder;
import frc.robot.subsystems.roller.VelocityRoller;

public class CreateComponents {

	public static VelocityPositionArm createTurret() {
		ArmSimulationConstants turretSimulationConstants = new ArmSimulationConstants(
			TurretConstants.MAX_POSITION,
			TurretConstants.MIN_POSITION,
			TurretConstants.MIN_POSITION,
			TurretConstants.MOMENT_OF_INERTIA,
			TurretConstants.TURRET_RADIUS
		);
		return TalonFXArmBuilder.buildVelocityPositionArm(
			TurretConstants.LOG_PATH,
			IDs.TalonFXIDs.TURRET,
			TurretConstants.IS_INVERTED,
			TurretConstants.IS_CONTINUOUS_WRAP,
			TurretConstants.TALON_FX_FOLLOWER_CONFIG,
			TurretConstants.SYS_ID_ROUTINE_CONFIG,
			TurretConstants.FEEDBACK_CONFIGS,
			TurretConstants.REAL_SLOTS_CONFIG,
			TurretConstants.SIMULATION_SLOTS_CONFIG,
			TurretConstants.CURRENT_LIMIT,
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			TurretConstants.ARBITRARY_FEED_FORWARD,
			TurretConstants.FORWARD_SOFTWARE_LIMIT,
			TurretConstants.BACKWARDS_SOFTWARE_LIMIT,
			turretSimulationConstants
		);
	}

	public static IDigitalInput createTurretResetCheckSensor() {
		return Robot.ROBOT_TYPE.isReal()
			? new ChanneledDigitalInput(
				new DigitalInput(IDs.DigitalInputsIDs.TURRET_RESET_SENSOR),
				new Debouncer(TurretConstants.RESET_CHECK_SENSOR_DEBOUNCE_TIME),
				TurretConstants.IS_RESET_CHECK_SENSOR_INVERTED
			)
			: new ChooserDigitalInput("turretResetCheck");
	}

	public static FlyWheel createFlyWheel() {
		return KrakenX60FlyWheelBuilder.build("Subsystems/FlyWheel", IDs.TalonFXIDs.FLYWHEEL);
	}

	public static Arm createFourBar() {
		ArmSimulationConstants fourBarSimConstant = new ArmSimulationConstants(
			FourBarConstants.MAXIMUM_POSITION,
			FourBarConstants.MINIMUM_POSITION,
			FourBarConstants.MAXIMUM_POSITION,
			FourBarConstants.MOMENT_OF_INERTIA,
			FourBarConstants.FOUR_BAR_LENGTH
		);
		return TalonFXArmBuilder.buildDynamicMotionMagicArm(
			FourBarConstants.LOG_PATH,
			IDs.TalonFXIDs.FOUR_BAR,
			FourBarConstants.IS_INVERTED,
			FourBarConstants.IS_CONTINUOUS_WRAP,
			FourBarConstants.TALON_FX_FOLLOWER_CONFIG,
			FourBarConstants.SYS_ID_ROUTINE,
			FourBarConstants.FEEDBACK_CONFIGS,
			FourBarConstants.REAL_SLOT,
			FourBarConstants.SIMULATION_SLOT,
			FourBarConstants.CURRENT_LIMIT,
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			FourBarConstants.ARBITRARY_FEED_FORWARD,
			FourBarConstants.FORWARD_SOFTWARE_LIMITS,
			FourBarConstants.BACKWARD_SOFTWARE_LIMITS,
			fourBarSimConstant,
			FourBarConstants.MAX_ACCELERATION_RPS_SQUARE,
			FourBarConstants.MAX_VELOCITY_RPS
		);
	}

	public static IDigitalInput createFourBarSensorResetCheck() {
		return Robot.ROBOT_TYPE.isReal()
			? new ChanneledDigitalInput(
				new DigitalInput(IDs.DigitalInputsIDs.FOUR_BAR_RESET_SENSOR),
				new Debouncer(FourBarConstants.RESET_CHECK_SENSOR_DEBOUNCE_TIME),
				FourBarConstants.IS_RESET_CHECK_SENSOR_INVERTED
			)
			: new ChooserDigitalInput("intakeResetCheck");
	}

	public static Arm createHood() {
		ArmSimulationConstants hoodSimulationConstants = new ArmSimulationConstants(
			HoodConstants.MAXIMUM_POSITION,
			HoodConstants.MINIMUM_POSITION,
			HoodConstants.MINIMUM_POSITION,
			HoodConstants.MOMENT_OF_INERTIA,
			HoodConstants.HOOD_LENGTH_METERS
		);
		return TalonFXArmBuilder.buildDynamicMotionMagicArm(
			RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/Hood",
			IDs.TalonFXIDs.HOOD,
			HoodConstants.IS_INVERTED,
			HoodConstants.IS_CONTINUOUS_WRAP,
			new TalonFXFollowerConfig(),
			HoodConstants.SYSIDROUTINE_CONFIG,
			HoodConstants.FEEDBACK_CONFIGS,
			HoodConstants.REAL_SLOT,
			HoodConstants.SIMULATION_SLOT,
			HoodConstants.CURRENT_LIMIT,
			RobotConstants.DEFAULT_SIGNALS_FREQUENCY_HERTZ,
			HoodConstants.ARBITRARY_FEEDFORWARD,
			HoodConstants.FORWARD_SOFTWARE_LIMIT,
			HoodConstants.BACKWARD_SOFTWARE_LIMIT,
			hoodSimulationConstants,
			HoodConstants.DEFAULT_MAX_ACCELERATION_PER_SECOND_SQUARE,
			HoodConstants.DEFAULT_MAX_VELOCITY_PER_SECOND
		);
	}

	public static Roller createIntakeRollers() {
		return SparkMaxRollerBuilder.build(
			RobotConstants.SUBSYSTEM_LOGPATH_PREFIX + "/IntakeRollers",
			IDs.SparkMAXIDs.INTAKE_ROLLERS,
			IntakeRollerConstants.IS_INVERTED,
			IntakeRollerConstants.GEAR_RATIO,
			IntakeRollerConstants.CURRENT_LIMIT,
			IntakeRollerConstants.MOMENT_OF_INERTIA
		);
	}

	public static IDigitalInput createHoodResetCheckSensor() {
		return Robot.ROBOT_TYPE.isReal()
			? new ChanneledDigitalInput(
				new DigitalInput(IDs.DigitalInputsIDs.HOOD_RESET_SENSOR),
				new Debouncer(HoodConstants.RESET_CHECK_SENSOR_DEBOUNCE_TIME),
				HoodConstants.IS_RESET_CHECK_SENSOR_INVERTED
			)
			: new ChooserDigitalInput("hoodResetCheck");
	}

	public static VelocityRoller createTrain() {
		return TalonFXRollerBuilder.buildVelocityRoller(
			TrainConstant.LOG_PATH,
			IDs.TalonFXIDs.TRAIN,
			TrainConstant.REAL_SLOTS_CONFIG,
			TrainConstant.SIMULATION_SLOTS_CONFIG,
			TrainConstant.CURRENT_LIMIT,
			TrainConstant.FEEDBACK_CONFIGS,
			TrainConstant.MOMENT_OF_INERTIA,
			TrainConstant.IS_INVERTED
		);
	}

	public static Roller createBelly() {
		return SparkMaxRollerBuilder.build(
			BellyConstants.LOG_PATH,
			IDs.SparkMAXIDs.BELLY,
			BellyConstants.IS_INVERTED,
			BellyConstants.GEAR_RATIO,
			BellyConstants.CURRENT_LIMIT,
			BellyConstants.MOMENT_OF_INERTIA
		);
	}

}
