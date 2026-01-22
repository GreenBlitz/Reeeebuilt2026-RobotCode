package frc.utils.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import java.util.function.Function;
import java.util.function.Supplier;

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

	public static Matrix<N3, N1> applyDivisionFactorOnStandardDeviations(StandardDeviations2D currentStdDevs, Pair<Supplier<Boolean>, Double>... compensationApplying){
		double factor = 1;
		for(Pair<Supplier<Boolean>, Double> current : compensationApplying){
			if(current.getFirst().get()) factor *= current.getSecond();
		}
		return currentStdDevs.asColumnVector().div(factor);
	}

}
