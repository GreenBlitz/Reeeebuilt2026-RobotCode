package frc.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Optional;

public class optimalPitchCalculation {

	private final double limit;
	private final double important;
	private final double minX;
	private final double tolerance;

	private final double tagHeight1;
	private final double tagHeight2;
	private final double cameraHeight;
	private final double cameraRelativeTagHeight1;
	private final double cameraRelativeTagHeight2;

	private final double fovUp;
	private final double fovDown;

	private final double rangeLimit1;
	private final double rangeLimit2;
	private final double pitchRangeForTag1;
	private final double pitchRangeForTag2;
	private final double pitchRangeFor2Tags;

	public optimalPitchCalculation(
		double limit,
		double important,
		double minX,
		double tolerance,
		double tagHeight1,
		double tagHeight2,
		double cameraHeight,
		double fovUp,
		double fovDown
	) {
		this.limit = limit;
		this.important = important;
		this.minX = minX;
		this.tolerance = tolerance;

		this.tagHeight1 = tagHeight1;
		this.tagHeight2 = tagHeight2;
		this.cameraHeight = cameraHeight;
		this.cameraRelativeTagHeight1 = this.tagHeight1 - this.cameraHeight + this.tolerance;
		this.cameraRelativeTagHeight2 = this.tagHeight2 - this.cameraHeight + this.tolerance;

		this.fovUp = fovUp;
		this.fovDown = fovDown;

		this.rangeLimit1 = cameraRelativeTagHeight1
			/ Math.tan(
				Rotation2d.fromDegrees(calculateOptimalPitch(cameraRelativeTagHeight1, this.limit).getDegrees() + this.fovUp + this.fovDown)
					.getRadians()
			);
		this.rangeLimit2 = cameraRelativeTagHeight2
			/ Math.tan(
				Rotation2d.fromDegrees(calculateOptimalPitch(cameraRelativeTagHeight2, this.limit).getDegrees() + this.fovUp + this.fovDown)
					.getRadians()
			);
		this.pitchRangeForTag1 = this.limit - rangeLimit1;
		this.pitchRangeForTag2 = this.limit - rangeLimit2;
		this.pitchRangeFor2Tags = this.limit - minX;
	}

	private Rotation2d calculateOptimalPitch(double height, double x) {
		return Rotation2d.fromRadians(Math.atan2(height, x));
	}

	private double calculateRangeStartPoint(double rangeLimit, double pitchRangeForTag) {
		if (important > rangeLimit)
			return limit - pitchRangeForTag;
		return important;
	}

	private double calculateRangeEndPoint(double rangeLimit, double pitchRangeForTag) {
		if (important < rangeLimit)
			return important + pitchRangeForTag;
		return limit;
	}

	public double calculateOptimalPitchFor1Tag(double cameraRelativeTagHeight, double rangeLimit, double pitchRangeForTag) {
		return calculateOptimalPitch(cameraRelativeTagHeight, calculateRangeStartPoint(rangeLimit, pitchRangeForTag)).getDegrees() - fovUp;
	}

	private Optional<Double> calculateOptimalSharedPitch(double x) {
		if (
			Math.abs(
				calculateOptimalPitch(cameraRelativeTagHeight1, x).getDegrees() - calculateOptimalPitch(cameraRelativeTagHeight2, x).getDegrees()
			) <= fovUp
		) {
			return Optional.of(
				(calculateOptimalPitch(cameraRelativeTagHeight1, x).getDegrees()
					+ calculateOptimalPitch(cameraRelativeTagHeight2, x).getDegrees()) / 2
			);
		}
		return Optional.empty();
	}

	public double calculateOptimalPitchFor2Tags() {
		return calculateOptimalSharedPitch(calculateRangeStartPoint(minX, pitchRangeFor2Tags)).get();
	}

}
