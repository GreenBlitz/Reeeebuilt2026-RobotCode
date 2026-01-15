package frc.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Optional;

public class optimalPitchCalculation {

	private final double TAG_HEIGHT_1 = 1.124;
	private final double TAG_HEIGHT_2 = 0.55;
	private final double CAMERA_HEIGHT = 0.3;
	private final double HEIGHT_1;
	private final double HEIGHT_2;
	private final double LIMIT;
	private final double IMPORTANT;
	private final double FOV_UP;
	private final double FOV_DOWN;
	private final double max1;
	private final double max2;
	private final double range1;
	private final double range2;
	private final double rangeFinal;
	private final double minX;
	private final double TOLARANCE;

	public optimalPitchCalculation(
		double limit,
		double tagHeight1,
		double tagHeight2,
		double cameraHeight,
		double important,
		double fovUp,
		double fovDown,
		double minX,
		double tolarance
	) {
		this.LIMIT = limit;
		this.TAG_HEIGHT_1 = tagHeight1;
		this.TAG_HEIGHT_2 = tagHeight2;
		this.CAMERA_HEIGHT = cameraHeight;
		this.IMPORTANT = important;
		this.FOV_UP = fovUp;
		this.FOV_DOWN = fovDown;
		this.minX = minX;
		this.HEIGHT_1 = TAG_HEIGHT_1 - CAMERA_HEIGHT;
		this.HEIGHT_2 = TAG_HEIGHT_2 - CAMERA_HEIGHT;
		this.max1 = HEIGHT_1 / Math.tan(Rotation2d.fromDegrees(pitch(HEIGHT_1, LIMIT) + FOV_UP + FOV_DOWN).getRadians());
		this.max2 = HEIGHT_2 / Math.tan(Rotation2d.fromDegrees(pitch(HEIGHT_2, LIMIT) + FOV_UP + FOV_DOWN).getRadians());
		this.range1 = LIMIT - max1;
		this.range2 = LIMIT - max2;
		this.rangeFinal = LIMIT - minX;
		this.TOLARANCE = tolarance;
	}

	private double pitch(double height, double x) {
		return Rotation2d.fromRadians(Math.atan2(height, x)).getDegrees();
	}

	private double calculateRangeStartingPoint(double max, double range, double important, double limit) {
		if (important > max)
			return limit - range;
		return important;
	}

	private double calculateRangeEndingPoint(double max, double range, double important, double limit) {
		if (important < max)
			return important + range;
		return limit;
	}

	public double getOptimalPitchPer1Tag(
		double cameraRelativeTagHeight,
		double max,
		double range,
		double fovUp,
		double important,
		double limit
	) {
		return pitch(cameraRelativeTagHeight, calculateRangeStartingPoint(max, range, important, limit)) - fovUp;
	}

	private Optional<Double> sharedPitch(
		double x,
		double cameraRelativeTagHeight1,
		double cameraRelativeTagHeight2,
		double tolarance,
		double fovUp
	) {
		if (Math.abs(pitch(cameraRelativeTagHeight1 + tolarance, x) - pitch(cameraRelativeTagHeight2 + tolarance, x)) <= fovUp) {
			return Optional.of((pitch(cameraRelativeTagHeight1, x) + pitch(cameraRelativeTagHeight2, x)) / 2);
		}
		return Optional.empty();
	}

	public double optimalPitchPer2Tags(
		double minX,
		double rangeFinal,
		double important,
		double limit,
		double cameraRelativeTagHeight1,
		double cameraRelativeTagHeight2,
		double tolarance,
		double fovUp
	) {
		return sharedPitch(
			calculateRangeStartingPoint(minX, rangeFinal, important, limit),
			cameraRelativeTagHeight1,
			cameraRelativeTagHeight2,
			tolarance,
			fovUp
		).get();
	}

	public static void main(String[] args) {}

}