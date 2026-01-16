package frc.utils.math;

import edu.wpi.first.math.geometry.Rotation2d;

import java.util.Optional;

public class optimalPitchCalculation {

	private final double maxCameraXDistanceFromTag;
	private final double mustInRangeX;
	private final double pitchRangeEndX;

	private final double cameraRelativeTagHeight1;
	private final double cameraRelativeTagHeight2;

	private final double fov;

	private final double pitchRangeStartX1;
	private final double pitchRangeStartX2;
	private final double pitchRangeForTag1;
	private final double pitchRangeForTag2;
	private final double pitchRangeFor2Tags;

	public optimalPitchCalculation(
		double maxCameraXDistanceFromTag,
		double mustInRangeX,
		double pitchRangeEndX,
		double tagHeight1,
		double tagHeight2,
		double tagHeightTolerance,
		double cameraHeight,
		double fov
	) {
		this.maxCameraXDistanceFromTag = maxCameraXDistanceFromTag;
		this.mustInRangeX = mustInRangeX;
		this.pitchRangeEndX = pitchRangeEndX;

		this.cameraRelativeTagHeight1 = tagHeight1 - cameraHeight + tagHeightTolerance;
		this.cameraRelativeTagHeight2 = tagHeight2 - cameraHeight + tagHeightTolerance;

		this.fov = fov;

		this.pitchRangeStartX1 = cameraRelativeTagHeight1
			/ Math.tan(
				Rotation2d.fromDegrees(calculateOptimalPitch(cameraRelativeTagHeight1, this.maxCameraXDistanceFromTag).getDegrees() + this.fov)
					.getRadians()
			);
		this.pitchRangeStartX2 = cameraRelativeTagHeight2
			/ Math.tan(
				Rotation2d.fromDegrees(calculateOptimalPitch(cameraRelativeTagHeight2, this.maxCameraXDistanceFromTag).getDegrees() + this.fov)
					.getRadians()
			);
		this.pitchRangeForTag1 = this.maxCameraXDistanceFromTag - pitchRangeStartX1;
		this.pitchRangeForTag2 = this.maxCameraXDistanceFromTag - pitchRangeStartX2;
		this.pitchRangeFor2Tags = this.maxCameraXDistanceFromTag - pitchRangeEndX;
	}

	private Rotation2d calculateOptimalPitch(double cameraRelativeTagHeight, double cameraXDistanceFromTag) {
		return Rotation2d.fromRadians(Math.atan2(cameraRelativeTagHeight, cameraXDistanceFromTag));
	}

	private double calculateRangeStartPoint(double pitchRangeStartX, double pitchRangeForTag) {
		if (mustInRangeX > pitchRangeStartX)
			return maxCameraXDistanceFromTag - pitchRangeForTag;
		return mustInRangeX;
	}

	private double calculateRangeEndPoint(double pitchRangeStartX, double pitchRangeForTag) {
		if (mustInRangeX < pitchRangeStartX)
			return mustInRangeX + pitchRangeForTag;
		return maxCameraXDistanceFromTag;
	}

	public double calculateOptimalPitchFor1Tag(double cameraRelativeTagHeight, double pitchRangeStartX, double pitchRangeForTag) {
		return calculateOptimalPitch(cameraRelativeTagHeight, calculateRangeStartPoint(pitchRangeStartX, pitchRangeForTag)).getDegrees()
			- fov / 2;
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
		return calculateOptimalSharedPitch(calculateRangeStartPoint(pitchRangeEndX, pitchRangeFor2Tags)).get();
	}

}
