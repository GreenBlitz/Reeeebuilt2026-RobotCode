package frc.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Optional;

public class optimalPitchCalculation {

	private final double maxCameraXDistanceFromTag;
	private final double minCameraXDistanceFromTag;
	private final double mustInRangeX;

	private final double cameraRelativeTagHeight1;
	private final double cameraRelativeTagHeight2;

	private final Rotation2d fov;

	private final double xRangeFor2Tags;

	public optimalPitchCalculation(
		double maxCameraXDistanceFromTag,
		double mustInRangeX,
		double minCameraXDistanceFromTag,
		double tagHeight1,
		double tagHeight2,
		double tagHeightTolerance,
		double cameraHeight,
		double fovInDegrees
	) {
		this.maxCameraXDistanceFromTag = maxCameraXDistanceFromTag;
		this.mustInRangeX = mustInRangeX;
		this.minCameraXDistanceFromTag = minCameraXDistanceFromTag;

		this.cameraRelativeTagHeight1 = tagHeight1 - cameraHeight + tagHeightTolerance;
		this.cameraRelativeTagHeight2 = tagHeight2 - cameraHeight + tagHeightTolerance;

		this.fov = Rotation2d.fromDegrees(fovInDegrees);

		this.xRangeFor2Tags = this.maxCameraXDistanceFromTag - minCameraXDistanceFromTag;
	}

	private Rotation2d calculateOptimalPitch(double cameraRelativeTagHeight, double cameraXDistanceFromTag) {
		return Rotation2d.fromRadians(Math.atan2(cameraRelativeTagHeight, cameraXDistanceFromTag));
	}

	private double calculateRangeEndPoint(double xRangeStart, double xRangeForTag) {
		if (mustInRangeX < xRangeStart)
			return mustInRangeX + xRangeForTag;
		return maxCameraXDistanceFromTag;
	}

	public double calculateOptimalPitchFor1Tag(double cameraRelativeTagHeight, double xRangeStart, double xRangeForTag) {
		return calculateOptimalPitch(cameraRelativeTagHeight, calculateRangeEndPoint(xRangeStart, xRangeForTag)).getDegrees()
			- fov.getDegrees() / 2;
	}

	private Optional<Double> calculateOptimalSharedPitch(double cameraXDistanceFromTag) {
		if (
			Math.abs(
				calculateOptimalPitch(cameraRelativeTagHeight1, cameraXDistanceFromTag).getDegrees()
					- calculateOptimalPitch(cameraRelativeTagHeight2, cameraXDistanceFromTag).getDegrees()
			) <= fov.getDegrees()
		) {
			return Optional.of(
				(calculateOptimalPitch(cameraRelativeTagHeight1, cameraXDistanceFromTag).getDegrees()
					+ calculateOptimalPitch(cameraRelativeTagHeight2, cameraXDistanceFromTag).getDegrees()) / 2
			);
		}
		return Optional.empty();
	}

	public Optional<Double> calculateOptimalPitchFor2Tags() {
		return calculateOptimalSharedPitch(calculateRangeEndPoint(minCameraXDistanceFromTag, xRangeFor2Tags));
	}

}
