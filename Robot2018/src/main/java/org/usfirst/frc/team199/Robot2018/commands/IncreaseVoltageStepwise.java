package org.usfirst.frc.team199.Robot2018.commands;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;

public class IncreaseVoltageStepwise extends Command {
    private Drivetrain dt;
    private FileWriter fw;
    private String filename;
    private int voltage_index;
    private double[] volt_steps;


    public IncreaseVoltageStepwise(Drivetrain dt, double[] volt_steps, String filename) {
        requires(dt);
        this.dt = dt;
        this.volt_steps = volt_steps;
        this.filename = filename;
        voltage_index = 0;
    }

    @Override
	protected void initialize() {
        File f = new File(filename);
        try {
            f.createNewFile();
            fw = new FileWriter(f);
            fw.write("Timestamp (s),");
            fw.write("Voltage (V),");
            fw.write("LeftMotorVelocity (inches / s),");
            fw.write("RightMotorVelocity (inches / s)\n");
        } catch (IOException e) {
            System.out.println("Error caught creating FileWriter object: " + e);
        }
    }

    @Override
	protected void execute() {
        dt.suppliedVoltage = volt_steps[voltage_index];
        if (Robot.getBool("Arcade Drive Default Setup", true)) {
			dt.arcadeDrive(dt.suppliedVoltage / dt.maximumVoltage, 0.0);
        }
            
         dt.writeMeasuredVelocity(fw);
    }

    @Override
    // Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
        if (dt.getLeftEncRate() >= 0.75 * dt.getCurrentMaxSpeed() || dt.getRightEncRate() >= 0.75 * dt.getCurrentMaxSpeed()) {
            return true;
        } else {
            return false;
        }
    }

    @Override
	protected void end() {
        try {
            fw.close();
        } catch (IOException e) {
            System.out.println("Cannot close FileWriter");
        }
    }

    @Override
	protected void interrupted() {
        end();
    }

    public void increaseVoltageIndex() {
        voltage_index += 1;
    }
}