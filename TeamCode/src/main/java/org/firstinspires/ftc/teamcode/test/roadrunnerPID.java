//package org.firstinspires.ftc.teamcode.test;
//
//import com.acmerobotics.roadrunner.control.PIDCoefficients;
//import com.acmerobotics.roadrunner.control.PIDFController;
//
//public class roadrunnerPID {
//
//    // specify coefficients/gains
//    double kP;
//    double kI;
//    PIDCoefficients coeffs = new PIDCoefficients(kP, kI, kD);
//    // create the controller
//    PIDFController controller = new PIDFController(coeffs);
//
//    controller.setInputBounds(0.0, 2.0 * Math.PI);
//
//    // specify the setpoint
//    controller.setTargetPosition(setpoint);
//
//    // in each iteration of the control loop
//    // measure the position or output variable
//    // apply the correction to the input variable
//    double correction = controller.update(measuredPosition);
//}
