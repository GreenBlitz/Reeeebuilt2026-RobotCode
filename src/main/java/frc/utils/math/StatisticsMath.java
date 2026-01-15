package frc.utils.math;

import edu.wpi.first.math.geometry.Translation2d;

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

	public static <T extends Number> T getMajority(T[] arr, double tolerance) {
		return ToleranceMath.isNear(arr[0], arr[1], tolerance) || ToleranceMath.isNear(arr[0].doubleValue(), arr[2].doubleValue(), tolerance)
			? arr[0]
			: arr[1];
	}

	public static Translation2d getMajority(Translation2d[] arr, double tolerance) { //assuming its four modules
		if (ToleranceMath.isNear(arr[0], arr[1], tolerance)
				|| ToleranceMath.isNear(arr[0], arr[2], tolerance)) {
			return arr[0];
		} else {
			if (!ToleranceMath.isNear(arr[2], arr[1], tolerance))
				return arr[3];
			return arr[1];
		}
	}
}

