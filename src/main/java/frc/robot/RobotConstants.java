package frc.robot;


public class RobotConstants {

	public static final double DEFAULT_SIGNALS_FREQUENCY_HERTZ = 60;

	public static final String SUBSYSTEM_LOGPATH_PREFIX = "Subsystems";


	// data fetched from:
	// https://www.mt.com/mt_ext_files/Editorial/Generic/7/NewtonFactor_Editorial-Generic_1149155213028_files/Newton%20Factor%20and%20Values.pdf
	public static final double ISRAEL_G = 9.7933;

	// data calculated in Beaumont Texas by Lamar university:
	// https://bgi.obs-mip.fr/data-products/gravity-databases/absolute-gravity-data/#/explorer
	public static final double TEXAS_G = 9.79306;

	public static final double LOCAL_G = ISRAEL_G;

	public static final double ROBOT_MASS_KG = 45;
	public static final double ROBOT_MOI = 4.5;
	public static final double WHEEL_COF = 1;

	public static final double MAX_TIME_FOR_BPS_INTERPOLATOR = 35;
	public static final double TIME_FOR_AVERAGE_BPS_CALCULATION_SECONDS = 1;

	public static final double DISTANCE_FROM_ROBOT_CENTER_TO_HOPPER_EDGE_WHEN_OPENED_METERS = 0.65;

}
