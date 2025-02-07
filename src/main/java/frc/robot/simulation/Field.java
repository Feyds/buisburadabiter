package frc.robot.simulation;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class Field extends Field2d {
    public static Field field;

    public static Field getInstance() {
        if(field == null) {
            field = new Field();
        }
        return field;
    }

    private Field() {
        super();
    }
}
