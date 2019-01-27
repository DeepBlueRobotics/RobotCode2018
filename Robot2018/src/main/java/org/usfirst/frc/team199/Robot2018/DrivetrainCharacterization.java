package org.usfirst.frc.team199.Robot2018;

import java.util.ArrayList;
import java.util.Scanner;
import java.util.regex.*; 
import java.io.File;
import java.io.FileNotFoundException;

public class DrivetrainCharacterization {
    public static void main(String[] args) {
        DrivetrainCharacterization.simpleRegression(args[0], args[1]);
    }

    public static void multiVariateRegression(String file1, String file2) {

    }

    public static void simpleRegression(String file1, String file2) {
        // Argument 1 should be the filepath of the linear increase CSV file.
        // Argument 2 should be the filepath of the stepwise increase CSV file.
        double applied_voltage = 6.0;   // The voltage expected to be applied during the Stepwise Testing.
        double kv = 0.0;                // The constant parameter for velocity in the drivetrain characterization formula
        double ka = 0.0;                // The constant parameter for acceleration in the drivetrain characterization formula
        double voltage_intercept = 0.0;
        int spread = 18;                // How much data should our difference quotient cover.

        ArrayList<Double> linear_velocities = new ArrayList<Double>();
        ArrayList<Double> linear_voltages = new ArrayList<Double>();
        ArrayList<Double> stepwise_x = new ArrayList<Double>();
        ArrayList<Double> stepwise_acceleration = new ArrayList<Double>();
        double[] params = new double[2];

        try {
            Scanner filereader1 = new Scanner(new File(file1));
            while(filereader1.hasNext()) {
                String line1 = filereader1.nextLine();
                if(!(line1.equals("Timestamp (s),Voltage (V),LeftMotorVelocity (inches / s),RightMotorVelocity (inches / s)"))) {      // If the line does not contain characters
                    linear_voltages.add(Double.valueOf(line1.split(",")[1]));    // Append voltage
                    linear_velocities.add(0.5 * (Double.valueOf(line1.split(",")[2]) - Double.valueOf(line1.split(",")[3]))); // Append average of the left and right motor velocities
                }
            }
            filereader1.close();
        } catch (FileNotFoundException e) {
            System.out.println("The file being referenced may not exist. Error: " + e.toString());
        }

        /*double[] a = {1.47,1.50,1.52,1.55,1.57,1.60,1.63,1.65,1.68,1.70,1.73,1.75,1.78,1.80,1.83};
        double[] b = {52.21,53.12,54.48,55.84,57.20,58.57,59.93,61.29,63.11,64.47,66.28,68.10,69.92,72.19,74.46};
        ArrayList<Double> A = new ArrayList<Double>(a.length);
        ArrayList<Double> B = new ArrayList<Double>(a.length);
        for(int i = 0; i < a.length; i++) {
            A.add(a[i]);
            B.add(b[i]);
        }

        System.out.println(DrivetrainCharacterization.simpleRegression(A, B)[0]);*/

        //System.out.println(linear_velocities);
        params = DrivetrainCharacterization.simpleRegressionFormula(linear_voltages, linear_velocities);
        kv = 1 / params[1];
        voltage_intercept = -params[0] / params[1];

        try {
            Scanner filereader2 = new Scanner(new File(file2));
            ArrayList<Double> left_velocities = new ArrayList<Double>();
            ArrayList<Double> right_velocities = new ArrayList<Double>();
            while(filereader2.hasNext()) {
                String line2 = filereader2.nextLine();
                if(!(line2.equals("Timestamp (s),Voltage (V),LeftMotorVelocity (inches / s),RightMotorVelocity (inches / s)"))) {
                    double v1 = Double.valueOf(line2.split(",")[2]);
                    double v2 = Double.valueOf(line2.split(",")[3]);
                    double v3 = 0.5 * (v1 - v2);
                    //System.out.println(v1 + "," + v2);
                    //System.out.println(kv * v3 + voltage_intercept);
                    stepwise_x.add(applied_voltage - (kv * v3 + voltage_intercept));
                        
                    if (left_velocities.size() >= spread) {
                        double a1 = (v1 - left_velocities.get(left_velocities.size() - spread)) / (spread * 0.02);
                        double a2 = (v2 - right_velocities.get(right_velocities.size() - spread)) / (spread * 0.02);
                        //System.out.println(a1 + "," + a2);
                        stepwise_acceleration.add((Math.abs(a1) + Math.abs(a2)));
                    }

                    left_velocities.add(v1);
                    right_velocities.add(v2);
                }
            }
            filereader2.close();
        } catch (FileNotFoundException e) {
            System.out.println("The file being referenced may not exist. Error: " + e.toString());
        }

        for(int i = 0; i < spread; i++) {
            stepwise_x.remove(stepwise_x.size() - 1);
        }

        //System.out.println(stepwise_x);
        //System.out.println(stepwise_acceleration);
        params = DrivetrainCharacterization.simpleRegressionFormula(stepwise_x, stepwise_acceleration);
        ka = 1 / params[1];

        System.out.println("Velocity Constant is " + Double.toString(12 * kv) + " and the Acceleration Constant is " + Double.toString(12 * ka));
    };

    public static double[] simpleRegressionFormula(ArrayList<Double> xs, ArrayList<Double> ys) {
        double sx, sxx, sy, sxy, a, b;
        sx = sxx = sy = sxy = a = b = 0.0;
        int n = xs.size();
        
        for (int i = 0; i < n; i++) {
            sx += xs.get(i);
            sxx += xs.get(i) * xs.get(i);
            sxy += xs.get(i) * ys.get(i);
            sy += ys.get(i);
        }

        b = (n * sxy - sx * sy) / (n * sxx - sx * sx);
        a = (sy - b * sx) / n;

        double[] params = {a,b};
        return params;
    }
}