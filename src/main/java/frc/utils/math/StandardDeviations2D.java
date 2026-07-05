package frc.utils.math;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public record StandardDeviations2D(double xStandardDeviations, double yStandardDeviations, double angleStandardDeviations) {

	public StandardDeviations2D() {
		this(0, 0, 0);
	}

	public StandardDeviations2D(double standardDeviation) {
		this(standardDeviation, standardDeviation, standardDeviation);
	}

	public StandardDeviations2D(StandardDeviations2D standardDeviations2D) {
		this(standardDeviations2D.xStandardDeviations, standardDeviations2D.yStandardDeviations, standardDeviations2D.angleStandardDeviations);
	}

	public StandardDeviations2D(Matrix<N3, N1> columnVector) {
		this(columnVector.get(0, 0), columnVector.get(1, 0), columnVector.get(2, 0));
	}

	public Matrix<N3, N1> asColumnVector() {
		return VecBuilder.fill(xStandardDeviations, yStandardDeviations, angleStandardDeviations);
	}

	public double[] asPoseArray() {
		return new double[] {xStandardDeviations, yStandardDeviations, angleStandardDeviations};
	}

}
