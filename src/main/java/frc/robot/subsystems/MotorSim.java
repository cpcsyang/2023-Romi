// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MotorSim extends SubsystemBase {
    private int speed;
    private int order;  // motor order number
    private static int numMotors = 0;

    /** Creates a new MotorSim. */
    public MotorSim() {
        reset();
        order = numMotors;
        numMotors++;
    }

    /**
     * sets speed to 0
     */
    public void reset() {
        set(0);
    }

    public void set(int speed){
        this.speed = speed;
        if (speed < 0) speed = 0;
        if (speed > 100) speed = 100;
    }

    public String toString() {
        String str = "";
        for (int speed = 0; speed <= 100; speed++) {
            int strLen = 25;
            for (int i = 0; i <= 100; i += 100/strLen) {
                if (speed > i) {
                    str += "=";
                } else {
                    str += "-";
                }
            }
            str = str.substring(0, str.length() - 1);
        }
        return "motor[" + order + "]:[" + str + "]";
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
