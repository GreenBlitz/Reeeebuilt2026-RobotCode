package frc.robot.subsystems.constants.flywheel;


import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.IDs;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.robot.subsystems.flywheel.KrakenX60FlyWheelBuilder;

public class FlywheelConstants {

	public static final double SENSOR_TO_MECHANISM_RATIO_MASTER = 40.0 / 36.0;
	public static final double SENSOR_TO_MECHANISM_RATIO_FOLLOWER = 40.0 / 36.0;

	public static final InvertedValue IS_MASTER_INVERTED = InvertedValue.Clockwise_Positive;
	public static final InvertedValue IS_FOLLOWER_INVERTED = InvertedValue.Clockwise_Positive;

	public static final double kP = 12;
	public static final double kI = 2;
	public static final double kD = 0;
	public static final double kV = 0.0;
	public static final double kS = 0.0;
	public static final double kA = 0.0;

	public static final double kP_SIM = 0.4;
	public static final double kI_SIM = 0;
	public static final double kD_SIM = 0;
	public static final double kS_SIM = 0.0;
	public static final double kV_SIM = 0.1385;
	public static final double kA_SIM = 0;

	public static final double MOMENT_OF_INERTIA = 0.01;
	public static final int CURRENT_LIMIT = 40;

	public static final Rotation2d MIN_ERROR_TO_APPLY_BANG_BANG_CONTROL_RPS = Rotation2d.fromRotations(3);

	public static FlyWheel createFlyWheel() {
		return KrakenX60FlyWheelBuilder.build("Subsystems/FlyWheel", IDs.TalonFXIDs.FLYWHEEL);
	}

}
