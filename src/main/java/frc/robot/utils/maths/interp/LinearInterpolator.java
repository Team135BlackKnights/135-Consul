package frc.robot.utils.maths.interp;

import java.util.List;

public class LinearInterpolator {
	private final List<Double> inputValues;
	private final List<Double> outputValues;

	public LinearInterpolator(List<Double> inputValues,
			List<Double> outputValues) {
		if (inputValues.size() != outputValues.size()) {
			throw new IllegalArgumentException(
					"Input and output lists must have the same size.");
		}
		this.inputValues = inputValues;
		this.outputValues = outputValues;
	}

	public double interp(double val) {
		// If the input value is out of bounds, return an appropriate value (could be the closest endpoint)
		if (val <= inputValues.get(0)) {
			return outputValues.get(0);
		}
		if (val >= inputValues.get(inputValues.size() - 1)) {
			return outputValues.get(outputValues.size() - 1);
		}
		// Find the interval [x0, x1] for the input value
		for (int i = 0; i < inputValues.size() - 1; i++) {
			if (val >= inputValues.get(i) && val <= inputValues.get(i + 1)) {
				// Perform linear interpolation
				double x0 = inputValues.get(i);
				double x1 = inputValues.get(i + 1);
				double y0 = outputValues.get(i);
				double y1 = outputValues.get(i + 1);
				// Calculate the interpolated value
				return y0 + (val - x0) * (y1 - y0) / (x1 - x0);
			}
		}
		// Should never reach here due to earlier checks
		throw new IllegalStateException("Interpolation failed.");
	}
}