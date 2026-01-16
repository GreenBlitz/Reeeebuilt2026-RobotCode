package frc.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Optional;

public class optimalPitchCalculation {

	private final double maxCameraXDistanceFromTag;
	private final double mustInRangeX;
	private final double xRangeStartFor2Tags;

	private final double cameraRelativeTagHeight1;
	private final double cameraRelativeTagHeight2;

	private final double fov;

	private final double xRangeStart1;
	private final double xRangeStart2;
	private final double xRangeForTag1;
	private final double xRangeForTag2;
	private final double xRangeFor2Tags;

	public optimalPitchCalculation(
		double maxCameraXDistanceFromTag,
		double mustInRangeX,
		double xRangeStartFor2Tags,
		double tagHeight1,
		double tagHeight2,
		double tagHeightTolerance,
		double cameraHeight,
		double fov
	) {
		this.maxCameraXDistanceFromTag = maxCameraXDistanceFromTag;
		this.mustInRangeX = mustInRangeX;
		this.xRangeStartFor2Tags = xRangeStartFor2Tags;

		this.cameraRelativeTagHeight1 = tagHeight1 - cameraHeight + tagHeightTolerance;
		this.cameraRelativeTagHeight2 = tagHeight2 - cameraHeight + tagHeightTolerance;

		this.fov = fov;

		this.xRangeStart1 = cameraRelativeTagHeight1
			/ Math.tan(
				Rotation2d.fromDegrees(calculateOptimalPitch(cameraRelativeTagHeight1, this.maxCameraXDistanceFromTag).getDegrees() + this.fov)
					.getRadians()
			);
		this.xRangeStart2 = cameraRelativeTagHeight2
			/ Math.tan(
				Rotation2d.fromDegrees(calculateOptimalPitch(cameraRelativeTagHeight2, this.maxCameraXDistanceFromTag).getDegrees() + this.fov)
					.getRadians()
			);
		this.xRangeForTag1 = this.maxCameraXDistanceFromTag - xRangeStart1;
		this.xRangeForTag2 = this.maxCameraXDistanceFromTag - xRangeStart2;
		this.xRangeFor2Tags = this.maxCameraXDistanceFromTag - xRangeStartFor2Tags;
	}

	private Rotation2d calculateOptimalPitch(double cameraRelativeTagHeight, double cameraXDistanceFromTag) {
		return Rotation2d.fromRadians(Math.atan2(cameraRelativeTagHeight, cameraXDistanceFromTag));
	}

	private double calculateRangeStartPoint(double xRangeStart, double xRangeForTag) {
		if (mustInRangeX > xRangeStart)
			return maxCameraXDistanceFromTag - xRangeForTag;
		return mustInRangeX;
	}

	private double calculateRangeEndPoint(double xRangeStart, double xRangeForTag) {
		if (mustInRangeX < xRangeStart)
			return mustInRangeX + xRangeForTag;
		return maxCameraXDistanceFromTag;
	}

	public double calculateOptimalPitchFor1Tag(double cameraRelativeTagHeight, double xRangeStart, double xRangeForTag) {
		return calculateOptimalPitch(cameraRelativeTagHeight, calculateRangeStartPoint(xRangeStart, xRangeForTag)).getDegrees() - fov / 2;
	}

	private Optional<Double> calculateOptimalSharedPitch(double cameraXDistanceFromTag) {
		if (
			Math.abs(
				calculateOptimalPitch(cameraRelativeTagHeight1, cameraXDistanceFromTag).getDegrees()
					- calculateOptimalPitch(cameraRelativeTagHeight2, cameraXDistanceFromTag).getDegrees()
			) <= fov
		) {
			return Optional.of(
				(calculateOptimalPitch(cameraRelativeTagHeight1, cameraXDistanceFromTag).getDegrees()
					+ calculateOptimalPitch(cameraRelativeTagHeight2, cameraXDistanceFromTag).getDegrees()) / 2
			);
		}
		return Optional.empty();
	}

	public double calculateOptimalPitchFor2Tags() {
		return calculateOptimalSharedPitch(calculateRangeStartPoint(xRangeStartFor2Tags, xRangeFor2Tags)).get();
	}

}
