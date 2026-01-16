package frc.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Optional;

public class optimalPitchCalculation {

	private final double maxCameraXDistanceFromTag;
	private final double minCameraXDistanceFromTag;
	private final double mustInRangeX;

	private final double cameraRelativeTag1Height;
	private final double cameraRelativeTag2Height;

	private final Rotation2d cameraFov;
	private final double xRangeFor2Tags;

	public optimalPitchCalculation(
		double maxCameraXDistanceFromTag,
		double mustInRangeX,
		double minCameraXDistanceFromTag,
		double tagHeight1,
		double tagHeight2,
		double tagHeightTolerance,
		double cameraHeight,
		double cameraFovInDegrees
	) {
		this.maxCameraXDistanceFromTag = maxCameraXDistanceFromTag;
		this.mustInRangeX = mustInRangeX;
		this.minCameraXDistanceFromTag = minCameraXDistanceFromTag;

		this.cameraRelativeTag1Height = tagHeight1 - cameraHeight + tagHeightTolerance;
		this.cameraRelativeTag2Height = tagHeight2 - cameraHeight + tagHeightTolerance;

		this.cameraFov = Rotation2d.fromDegrees(cameraFovInDegrees);

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

	private Optional<Double> calculateOptimalSharedPitch(double cameraXDistanceFromTag) {
		if (
			Math.abs(
				calculateOptimalPitch(cameraRelativeTag1Height, cameraXDistanceFromTag).getDegrees()
					- calculateOptimalPitch(cameraRelativeTag2Height, cameraXDistanceFromTag).getDegrees()
			) <= cameraFov.getDegrees()
		) {
			return Optional.of(
				(calculateOptimalPitch(cameraRelativeTag1Height, cameraXDistanceFromTag).getDegrees()
					+ calculateOptimalPitch(cameraRelativeTag2Height, cameraXDistanceFromTag).getDegrees()) / 2
			);
		}
		return Optional.empty();
	}

	public double calculateOptimalPitchFor1Tag(double cameraRelativeTagHeight, double xRangeStart, double xRangeForTag) {
		return calculateOptimalPitch(cameraRelativeTagHeight, calculateRangeEndPoint(xRangeStart, xRangeForTag)).getDegrees()
			- cameraFov.getDegrees() / 2;
	}

	public Optional<Double> calculateOptimalPitchFor2Tags() {
		return calculateOptimalSharedPitch(calculateRangeEndPoint(minCameraXDistanceFromTag, xRangeFor2Tags));
	}

}
