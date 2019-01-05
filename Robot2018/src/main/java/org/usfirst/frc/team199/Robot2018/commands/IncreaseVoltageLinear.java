package org.usfirst.frc.team199.Robot2018.commands;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import org.usfirst.frc.team199.Robot2018.Robot;
import org.usfirst.frc.team199.Robot2018.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.command.Command;

/* Command which supplies the motors with a desired voltage that increases linearly.
*/
public class IncreaseVoltageLinear extends Command {
    private Drivetrain dt;
    private FileWriter fw;
    private String filename;
    private double volt_step;

	public IncreaseVoltageLinear(Drivetrain dt, double volt_step, String filename) {
        requires(dt);
        this.dt = dt;
        this.volt_step = volt_step;
        this.filename = filename;
    }

	// Called just before this Command runs the first time
	@Override
	protected void initialize() {
        File f = new File(filename);
        try {
            f.createNewFile();
            fw = new FileWriter(f);
            fw.write("Timestamp (s),");
            fw.write("LeftMotorVelocity (inches / s),");
            fw.write("RightMotorVelocity (inches / s)\n");
        } catch (IOException e) {
            System.out.println("FileWriter object could not be created. Does the file with the associated filename exist?");
        }
	}

	// Called repeatedly when this Command is scheduled to run
	@Override
	protected void execute() {
        if (dt.suppliedVoltage + volt_step <= dt.maximumVoltage) {
            dt.suppliedVoltage += volt_step;

            if (Robot.getBool("Arcade Drive Default Setup", true)) {
				dt.arcadeDrive(dt.suppliedVoltage / dt.maximumVoltage, 0.0);
            }
            
            dt.writeMeasuredVelocity(fw);
        }
	}

	// Make this return true when this Command no longer needs to run execute()
	@Override
	protected boolean isFinished() {
		if (dt.suppliedVoltage >= dt.maximumVoltage) {
            return true;
        }
        else {
            return false;
        }
	}

	// Called once after isFinished returns true
	@Override
	protected void end() {
        try {
            fw.close();
        } catch (IOException e) {
            System.out.println("Cannot close FileWriter");
        }
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	@Override
	protected void interrupted() {
		end();
	}
}
