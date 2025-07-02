package frc.robot.utils.selfCheck.vision;

import java.nio.channels.UnsupportedAddressTypeException;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;

import frc.robot.utils.selfCheck.SelfChecking;
import frc.robot.utils.selfCheck.SubsystemFault;
import frc.robot.utils.vision.LimelightHelpers;
import frc.robot.utils.vision.LimelightHelpers.LimelightResults;
/**
 * @apiNote Untested, needs to be checked with hardware
 */
public class SelfCheckingLimelight implements SelfChecking {
	private final String label;

	public SelfCheckingLimelight(String label) { this.label = label; }

	private CompletableFuture<LimelightResults> fetchResults() {
		return CompletableFuture
				.supplyAsync(() -> LimelightHelpers.getLatestResults(label))
				.completeOnTimeout(null, 200, TimeUnit.MILLISECONDS);
	}

	@Override
	public ConcurrentLinkedQueue<SubsystemFault> checkForFaults() {
		ConcurrentLinkedQueue<SubsystemFault> faultList = new ConcurrentLinkedQueue<>();
		CompletableFuture<LimelightResults> resultsFuture = fetchResults();
		resultsFuture.thenAccept(results -> {
			if (results != null && !results.error.isEmpty()) {
				faultList.add(new SubsystemFault(results.error, false, true));
			}
		}).exceptionally(e -> {
			faultList.add(new SubsystemFault(
					label + " Limelight not responding at ALL", false, true));
			return null;
		});
		return faultList;
	}

	/**
	 * This is not supported, and will throw an error. You are trying to access
	 * the physical hardware of the Limelight, which is not possible. Why would
	 * someone need this? -N
	 */
	@Override
	public Object getHardware() { throw new UnsupportedAddressTypeException(); }
}
