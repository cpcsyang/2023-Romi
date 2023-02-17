// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MotorSim;

public class RunMotorSim extends CommandBase {
    private MotorSim motor;
    private long cycleCount;
    private DoubleSupplier speed;

    /** Creates a new RunMotorSim. */
    public RunMotorSim(MotorSim subsystem, DoubleSupplier speed) {
        this.motor = subsystem;
        this.speed = speed;
        cycleCount = 0;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(motor);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        motor.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (cycleCount % 50 == 0) toString();
        cycleCount++;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        motor.reset();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
