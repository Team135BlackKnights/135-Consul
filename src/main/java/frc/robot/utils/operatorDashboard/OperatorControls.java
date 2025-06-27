package frc.robot.utils.operatorDashboard;

import org.littletonrobotics.junction.Logger;

import frc.robot.utils.VirtualSubsystem;

public class OperatorControls extends VirtualSubsystem {
    private final OperatorDashboardIO io;
    private final OperatorDashboardIOInputsAutoLogged inputs = new OperatorDashboardIOInputsAutoLogged();
    private int topRow = -1;
    private int midRow = -1;
    private int hybridRow = -1;
    private String targetBox = "";
    public OperatorControls(OperatorDashboardIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("OperatorControls", inputs);

        // Bitfields (print if changed)
        if (inputs.topRow.length > 0 && inputs.topRow[0] != topRow) {
            topRow = inputs.topRow[0];
            printGrid();
        }
        if (inputs.midRow.length > 0 && inputs.midRow[0] != midRow) {
            midRow = inputs.midRow[0];
            printGrid();
        }
        if (inputs.hybridRow.length > 0 && inputs.hybridRow[0] != hybridRow) {
            hybridRow = inputs.hybridRow[0];
            printGrid();
        }
        if (inputs.targetBox != null && !inputs.targetBox.isEmpty()) {
            if (!inputs.targetBox.equals(targetBox)) {
                targetBox = inputs.targetBox;
                System.out.println("TargetBox changed to: " + targetBox);
                // You can add extra logic here, e.g. update robot targeting
            }
        }
        // Echo values back to dashboard
        io.setTopRow(topRow);
        io.setMidRow(midRow);
        io.setHybridRow(hybridRow);
        io.setTargetBox(targetBox);
    }

    private void printGrid() {
        System.out.println("=== Operator Grid Update ===");
        System.out.println("Top Row    : " + formatRow(topRow, false));
        System.out.println("Middle Row : " + formatRow(midRow, false));
        System.out.println("Hybrid Row : " + formatRow(hybridRow, true));
        System.out.println("============================");
    }

    private String formatRow(int value, boolean isHybrid) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < 9; i++) { // left to right = LSB to MSB
            if (isHybrid) {
                int state = (value >> (i * 2)) & 0b11;
                switch (state) {
                    case 0b01: sb.append("[Algae] "); break;
                    case 0b10: sb.append("[Coral] "); break;
                    default:   sb.append("[____] "); break;
                }
            } else {
                sb.append(((value >> i) & 1) == 1 ? "[ X ] " : "[____] ");
            }
        }
        return sb.toString();
    }
}