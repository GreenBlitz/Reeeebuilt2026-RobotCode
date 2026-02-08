package frc.robot.subsystems.constants.flywheel;


import frc.robot.IDs;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.robot.subsystems.flywheel.KrakenX60FlyWheelBuilder;

public class FlywheelConstants {

	public final static double kP = 0.2;
	public final static double kI = 0;
	public final static double kD = 0;
	public final static double kV = 0.07367;
	public final static double kS = 0.22;
	public final static double kA = 0;

	public final static double kP_SIM = 0.4;
	public final static double kI_SIM = 0;
	public final static double kD_SIM = 0;
	public final static double kS_SIM = 0.0;
	public final static double kV_SIM = 0.1385;
	public final static double kA_SIM = 0;

	public final static double SENSOR_TO_MECHANISM_RATIO_MASTER = 1.0 / 1.5;
	public final static double SENSOR_TO_MECHANISM_RATIO_FOLLOWER = 1.0 / 1.5;

	public final static double MOMENT_OF_INERTIA = 0.01;
	public final static int CURRENT_LIMIT = 80;

	public static FlyWheel createFlyWheel() {
		return KrakenX60FlyWheelBuilder.build("Subsystems/FlyWheel", IDs.TalonFXIDs.FLYWHEEL);
	}

}
