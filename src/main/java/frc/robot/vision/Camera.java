package frc.robot.vision;


public enum Camera {

	LEFT,
	FRONT,
	RIGHT,
	NONE;

	public static Camera getCameraByName(String name) {
		return switch (name) {
			case "limelight-left" -> LEFT;
			case "limelight-front" -> FRONT;
			case "limelight-right" -> RIGHT;
			default -> NONE;
		};
	}

}
