package frc.utils.math;

import edu.wpi.first.math.Pair;
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

	public static <T extends Number> T getMajority(T[] arr, double tolerance) { //only one standout
		return ToleranceMath.isNear(arr[0], arr[1], tolerance) || ToleranceMath.isNear(arr[0].doubleValue(), arr[2].doubleValue(), tolerance)
			? arr[0]
			: arr[1];
	}

	public static Pair<Integer, Translation2d> getMajority(Translation2d[] arr, double tolerance) { //assuming its four translations
		if (arr.length != 4) {
			System.out.println("BROTHER EWWWW");
			return null;
		}
		Translation2d majority;
		if (ToleranceMath.isNear(arr[0], arr[1], tolerance)
				|| ToleranceMath.isNear(arr[0], arr[2], tolerance)) {
			majority = arr[0];
		} else {
			if (!ToleranceMath.isNear(arr[1], arr[2], tolerance))
				majority = arr[3];
			else
				majority =  arr[1];
		}
		int count = 0;
		for (int i = 0; i < arr.length; i++) {
			if(ToleranceMath.isNear(majority, arr[i], tolerance))
				count++;
		}
		return new Pair<>(count, majority);
	}
}

