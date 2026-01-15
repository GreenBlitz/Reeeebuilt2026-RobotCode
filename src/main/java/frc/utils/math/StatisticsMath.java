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

	public static Translation2d getMajority(Translation2d[] arr, double tolerance) {
		int countEqual = 1;
		int countNotEqual = 0;
		for (int i = 1; i < arr.length; i++) {
			if (ToleranceMath.isNear(arr[0], arr[i], tolerance))
				countEqual++;
			else
				countNotEqual++;
		}
		if (countEqual >= countNotEqual)
			return arr[0];
		for (int i = 1; i < arr.length; i++) {
			if (!ToleranceMath.isNear(arr[0], arr[i], tolerance))
				return arr[i];
		}
		return arr[0];
	}

}
