package frc.robot;

import frc.robot.hardware.phoenix6.BusChain;
import frc.robot.hardware.phoenix6.Phoenix6DeviceID;

public class IDs {

	public static class TalonFXIDs {

		public static final Phoenix6DeviceID SWERVE_FRONT_LEFT_STEER = new Phoenix6DeviceID(0, BusChain.CHASSIS);
		public static final Phoenix6DeviceID SWERVE_FRONT_LEFT_DRIVE = new Phoenix6DeviceID(1, BusChain.CHASSIS);
		public static final Phoenix6DeviceID SWERVE_FRONT_RIGHT_STEER = new Phoenix6DeviceID(2, BusChain.CHASSIS);
		public static final Phoenix6DeviceID SWERVE_FRONT_RIGHT_DRIVE = new Phoenix6DeviceID(3, BusChain.CHASSIS);
		public static final Phoenix6DeviceID SWERVE_BACK_LEFT_STEER = new Phoenix6DeviceID(4, BusChain.CHASSIS);
		public static final Phoenix6DeviceID SWERVE_BACK_LEFT_DRIVE = new Phoenix6DeviceID(5, BusChain.CHASSIS);
		public static final Phoenix6DeviceID SWERVE_BACK_RIGHT_STEER = new Phoenix6DeviceID(6, BusChain.CHASSIS);
		public static final Phoenix6DeviceID SWERVE_BACK_RIGHT_DRIVE = new Phoenix6DeviceID(7, BusChain.CHASSIS);

		public static final Phoenix6DeviceID FLYWHEEL = new Phoenix6DeviceID(10, BusChain.ROBORIO);
		public static final Phoenix6DeviceID FLYWHEEL_FOLLOWER = new Phoenix6DeviceID(11, BusChain.ROBORIO);

		public static final Phoenix6DeviceID HOOD = new Phoenix6DeviceID(20, BusChain.ROBORIO);

		public static final Phoenix6DeviceID TURRET = new Phoenix6DeviceID(30, BusChain.CHASSIS);

		public static final Phoenix6DeviceID TRAIN = new Phoenix6DeviceID(40, BusChain.ROBORIO);

		public static final Phoenix6DeviceID INTAKE_ROLLERS = new Phoenix6DeviceID(50, BusChain.CHASSIS);
		public static final Phoenix6DeviceID FOUR_BAR = new Phoenix6DeviceID(51, BusChain.CHASSIS);

		public static final Phoenix6DeviceID BELLY = new Phoenix6DeviceID(60, BusChain.ROBORIO);

	}

	public static class CANCoderIDs {

		public static final Phoenix6DeviceID SWERVE_FRONT_LEFT = new Phoenix6DeviceID(0, BusChain.CHASSIS);

		public static final Phoenix6DeviceID SWERVE_FRONT_RIGHT = new Phoenix6DeviceID(1, BusChain.CHASSIS);

		public static final Phoenix6DeviceID SWERVE_BACK_LEFT = new Phoenix6DeviceID(2, BusChain.CHASSIS);

		public static final Phoenix6DeviceID SWERVE_BACK_RIGHT = new Phoenix6DeviceID(3, BusChain.CHASSIS);

	}

	public static class Pigeon2IDs {

		public static final Phoenix6DeviceID SWERVE = new Phoenix6DeviceID(0, BusChain.ROBORIO);

	}

	public static class CANdleIDs {
	}

	public static class SparkMAXIDs {

	}

	public static class DigitalInputsIDs {

		public static final int FOUR_BAR_RESET_SENSOR = 11;

		public static final int TURRET_RESET_SENSOR = 12;

		public static final int HOOD_RESET_SENSOR = 7;

		public static final int TRAIN_BALL_SENSOR = 9;

	}

}
