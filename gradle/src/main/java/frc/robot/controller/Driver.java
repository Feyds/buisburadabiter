package frc.robot.controller;

import edu.wpi.first.wpilibj.XboxController;

public class Driver {
    XboxController controller;

    public Driver(int port) {
        controller = new XboxController(port);
    }

    public double getSpeed() {
        return -controller.getLeftY();
    }

    public double getRotation() {
        return -controller.getRightX();
    }
    
    public boolean getWantsToScoreCoral() {
        return controller.getAButton();
    }

    public boolean getWantsToIntakeAlgae() {
        return controller.getXButton();
    }

    public boolean getWantsToScoreAlgae() {
        return controller.getBButton();
    }
}
