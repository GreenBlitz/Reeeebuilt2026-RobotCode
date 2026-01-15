package frc.utils.math;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.swerve.SwerveConstants;

import java.util.function.Function;

public class StatisticsMath {

	public static <T extends Iterable<E>, E> double calculateStandardDeviations(T iterable, Function<E, Double> getValue) {
		int length = 0;
		double total = 0;
		for (E value : iterable) {
			length++;
			total += getValue.apply(value);
		}
		double mean = total / length;
		double deviationsTotal = 0;
		for (E value : iterable) {
			deviationsTotal += Math.pow(getValue.apply(value) - mean, 2);
		}
		return Math.sqrt(deviationsTotal / length);
	}

	public static <T extends Number> T getMajority(T[] arr) {
		return ToleranceMath.isNear(arr[0].doubleValue(), arr[1].doubleValue(), SwerveConstants.SKID_TOLERANCE_VELOCITY_METERS_PER_SECOND_MODULE_TO_MODULE)
			|| ToleranceMath.isNear(arr[0].doubleValue(), arr[2].doubleValue(), SwerveConstants.SKID_TOLERANCE_VELOCITY_METERS_PER_SECOND_MODULE_TO_MODULE)
				? arr[0]
				: arr[1];
	}

	public static Translation2d getMajority (Translation2d[] arr) {
		return ToleranceMath.isNear(arr[0], arr[1], SwerveConstants.SKID_TOLERANCE_VELOCITY_METERS_PER_SECOND_MODULE_TO_MODULE)
				|| ToleranceMath.isNear(arr[0], arr[2], SwerveConstants.SKID_TOLERANCE_VELOCITY_METERS_PER_SECOND_MODULE_TO_MODULE)
				? arr[0]
				: arr[1];
	}

}
