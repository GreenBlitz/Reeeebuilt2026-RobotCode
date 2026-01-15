package frc.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Optional;

public class optimalPitchCalculation {

	private final double TAG_HEIGHT_1;
	private final double TAG_HEIGHT_2;
	private final double CAMERA_HEIGHT;
	// tag1 = 1.124, tag2 = 0.55, cameraHeight = 0.3
	private final double CAMERA_RELATIVE_TAG_HEIGHT1;
	private final double CAMERA_RELATIVE_TAG_HEIGHT2;
	private final double limit;
	private final double important;
	private final double FOV_UP;
	private final double FOV_DOWN;
	private final double max1;
	private final double max2;
	private final double range1;
	private final double range2;
	private final double rangeFinal;
	private final double minX;
	private final double TOLERANCE;

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
		this.limit = limit;
		this.TAG_HEIGHT_1 = tagHeight1;
		this.TAG_HEIGHT_2 = tagHeight2;
		this.CAMERA_HEIGHT = cameraHeight;
		this.important = important;
		this.FOV_UP = fovUp;
		this.FOV_DOWN = fovDown;
		this.minX = minX;
		this.CAMERA_RELATIVE_TAG_HEIGHT1 = TAG_HEIGHT_1 - CAMERA_HEIGHT;
		this.CAMERA_RELATIVE_TAG_HEIGHT2 = TAG_HEIGHT_2 - CAMERA_HEIGHT;
		this.max1 = CAMERA_RELATIVE_TAG_HEIGHT1
			/ Math.tan(Rotation2d.fromDegrees(pitch(CAMERA_RELATIVE_TAG_HEIGHT1, this.limit) + FOV_UP + FOV_DOWN).getRadians());
		this.max2 = CAMERA_RELATIVE_TAG_HEIGHT2
			/ Math.tan(Rotation2d.fromDegrees(pitch(CAMERA_RELATIVE_TAG_HEIGHT2, this.limit) + FOV_UP + FOV_DOWN).getRadians());
		this.range1 = this.limit - max1;
		this.range2 = this.limit - max2;
		this.rangeFinal = this.limit - minX;
		this.TOLERANCE = tolarance;
	}

	private double pitch(double height, double x) {
		return Rotation2d.fromRadians(Math.atan2(height, x)).getDegrees();
	}

	private double calculateRangeStartingPoint(double max, double range) {
		if (important > max)
			return limit - range;
		return important;
	}

	private double calculateRangeEndingPoint(double max, double range) {
		if (important < max)
			return important + range;
		return limit;
	}

	public double getOptimalPitchPer1Tag(double cameraRelativeTagHeight, double max, double range) {
		return pitch(cameraRelativeTagHeight, calculateRangeStartingPoint(max, range)) - FOV_UP;
	}

	private Optional<Double> sharedPitch(double x) {
		if (Math.abs(pitch(CAMERA_RELATIVE_TAG_HEIGHT1 + TOLERANCE, x) - pitch(CAMERA_RELATIVE_TAG_HEIGHT2 + TOLERANCE, x)) <= FOV_UP) {
			return Optional.of((pitch(CAMERA_RELATIVE_TAG_HEIGHT1, x) + pitch(CAMERA_RELATIVE_TAG_HEIGHT2, x)) / 2);
		}
		return Optional.empty();
	}

	public double optimalPitchPer2Tags() {
		return sharedPitch(calculateRangeStartingPoint(minX, rangeFinal)).get();
	}

	public static void main(String[] args) {}

}
