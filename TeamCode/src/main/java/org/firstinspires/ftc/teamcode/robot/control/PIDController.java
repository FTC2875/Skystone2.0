//package org.firstinspires.ftc.teamcode.robot.control;
//
//
//import com.acmerobotics.roadrunner.control.PIDCoefficients;
//import com.acmerobotics.roadrunner.control.PIDFController;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//
//import static org.firstinspires.ftc.teamcode.robot.drivetrain.DriveConstants.kA;
//import static org.firstinspires.ftc.teamcode.robot.drivetrain.DriveConstants.kStatic;
//import static org.firstinspires.ftc.teamcode.robot.drivetrain.DriveConstants.kV;
//
///**
// * Usage: Implements and simplifies universal PID control.
// *
// *
// *
// * Author: Daniel
// */
//
//public class PIDController {
//    double input;
//    double output;
//    double error;
//    double setpoint;
//
//    double kP;
//    double kI;
//    double kD;
//
//    PIDFController controller = new PIDFController(coeffs, kV, kA, kStatic);
//
//    controller.setTargetPosition(position);
//    controller.setTargetVelocity(velocity);
//    controller.setTargetAcceleration(acceleration + g);
//
//    double correction = controller.update(measuredPosition);
//
//    private Telemetry telemetry;
//
//    public PIDController(Telemetry telemetry) {
//        this.telemetry = telemetry;
//    }
//        public double Compute(double input, double ouput) {
//        input = input;
//            return kP;
//        }
//    }
//
