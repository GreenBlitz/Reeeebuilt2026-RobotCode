package frc.utils.limelight;

public class LimelightHelpersAdditions {

	public static boolean getIsConnected(String limelightName) {
		return LimelightHelpers.getLimelightNTTable(limelightName).containsKey("getpipe");
	}

	public static double getTemperature(String limelightName){
		return LimelightHelpers.getLimelightNTTable(limelightName).getEntry("temp").getDouble(0);
	}

}
