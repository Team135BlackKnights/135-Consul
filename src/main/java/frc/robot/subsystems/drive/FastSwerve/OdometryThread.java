package frc.robot.subsystems.drive.FastSwerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import frc.robot.Constants;
import frc.robot.utils.CompetitionFieldUtils.Simulation.SwerveDriveSimulation;
import frc.robot.utils.drive.DriveConstants;

import org.littletonrobotics.junction.AutoLog;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.Supplier;

public interface OdometryThread {
	final class OdometryDoubleInput {
		private final Supplier<Double> supplier;
		private final Queue<Double> queue;

		public OdometryDoubleInput(Supplier<Double> signal) {
			this.supplier = signal;
			this.queue = new ArrayBlockingQueue<>(20);
		}

		public void cacheInputToQueue() { this.queue.offer(supplier.get()); }
	}

	List<OdometryDoubleInput> registeredInputs = new ArrayList<>();
	List<BaseStatusSignal> registeredStatusSignals = new ArrayList<>();

	static Queue<Double> registerSignalInput(StatusSignal<Double> signal) {
		signal.setUpdateFrequency(DriveConstants.TrainConstants.odomHz, .02);
		registeredStatusSignals.add(signal);
		return registerInput(signal.asSupplier());
	}

	static Queue<Double> registerInput(Supplier<Double> supplier) {
		final OdometryDoubleInput odometryDoubleInput = new OdometryDoubleInput(
				supplier);
		registeredInputs.add(odometryDoubleInput);
		return odometryDoubleInput.queue;
	}

	static OdometryThread createInstance() {
		return switch (Constants.currentMode) {
		case REAL -> new OdometryThreadReal(
				registeredInputs.toArray(new OdometryDoubleInput[0]),
				registeredStatusSignals.toArray(new BaseStatusSignal[0]));
		case SIM -> new SwerveDriveSimulation.OdometryThreadSim();
		case REPLAY -> inputs -> {
		};
		};
	}

	@AutoLog
	class OdometryThreadInputs {
		public double[] measurementTimeStamps = new double[0];
	}

	void updateInputs(OdometryThreadInputs inputs);

	default void start() {}

	default void lockOdometry() {}

	default void unlockOdometry() {}
}
