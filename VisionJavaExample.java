package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

@TeleOp(name="plsmergi", group="riffe")

public class VisionJavaExample extends LinearOpMode{
    MasterVision vision;
    SampleRandomizedPositions goldPosition;
    DcMotor left_f;
    DcMotor left_b;
    DcMotor right_f;
    DcMotor right_b;
    BNO055IMU imu;
    @Override
    public void runOpMode() throws InterruptedException {
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;// recommended camera direction
        parameters.vuforiaLicenseKey = "ASHFs7r/////AAABmbSiCe6Dq03flLd+mIyA0Hsp+FgwD3zZqmqX01p8K9eqFha+bq9YuPgTU4hPEPKUwxfYw1u5aYTuEtoXw/2shsdd7e1Cu0rLJ5iHnFlxwYLB1BFjZrIHHTqoNC6oh3nQy4SAwz7q9XULkx5IwZGVcZynRXDmdi+dWLxqovRMmQ5yTbI0Smh1IvMPFsPVGTVRqHZuRPVymZAFuUUD5aIsK0CuYjljG2DDCK+2NWq7flga/KCGx7OdaUUL8Jem9bfHbxmMbOv06+rPPWLCjy74c5CJtqTfL24U9tda2UB/v0KFMlVzKvzXEt0Et0OkIBcgn9S+aaV7xYy7vEJ25ofYwG4hEhmuLMsxwmtKRGDC3eDf";

        vision = new MasterVision(parameters, hardwareMap, true, MasterVision.TFLiteAlgorithm.INFER_RIGHT);
        vision.init();// enables the camera overlay. this will take a couple of seconds
        vision.enable();// enables the tracking algorithms. this might also take a little time
        left_b=hardwareMap.get(DcMotor.class,"left_b");
        left_f=hardwareMap.get(DcMotor.class,"left_f");
        right_b=hardwareMap.get(DcMotor.class,"right_b");
        right_f=hardwareMap.get(DcMotor.class,"right_f");

        right_b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        BNO055IMU.Parameters parameterss = new BNO055IMU.Parameters();
        parameterss.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameterss.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameterss.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameterss.loggingEnabled      = true;
        parameterss.loggingTag          = "IMU";
        parameterss.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameterss);
        double currentangle=imu.getAngularOrientation().firstAngle;
        waitForStart();
        double vs=0,vd=0;
        double pos=left_b.getCurrentPosition();
        goldPosition = vision.getTfLite().getLastKnownSampleOrder();

        while(opModeIsActive()){
            telemetry.addData("goldPosition was", goldPosition);// giving feedback
            double angle=imu.getAngularOrientation().firstAngle-currentangle;
            switch (goldPosition){ // using for things in the autonomous program
                case LEFT: {
                    telemetry.addLine("going to the left");
                       if(angle<=5)
                       {
                           vs=-0.2;
                           vd=0.2;
                           pos=left_b.getCurrentPosition();
                       }
                       else
                       {
                           if(left_b.getCurrentPosition()-pos>=-3500)
                           { vs=1;
                           vd=1;}
                           else
                           {
                               vs=0;
                               vd=0;
                           }
                       }
                }
                    break;
                case CENTER:{
                    telemetry.addLine("going straight");
                    if(left_b.getCurrentPosition()-pos>=-3800)
                    {
                        vs=1;
                        vd=1;
                    }
                    else
                    {
                        vs=0;
                        vd=0;
                    }
                    break;
                }
                case RIGHT: {
                    telemetry.addLine("going to the right");
                    if(angle>=-6)
                    {
                        vs=0.3;
                        vd=-0.3;
                    }
                    else
                    {
                        if(left_b.getCurrentPosition()-pos>=-3800)
                        {
                            vs=1;
                            vd=1;
                        }
                        else{
                        vs=0;
                        vd=0;}
                    }
                    break;
                }
                case UNKNOWN:
                    telemetry.addLine("staying put");
                    break;
            }
            left_b.setPower(-vs);
            left_f.setPower(-vs);
            right_b.setPower(vd);
            right_f.setPower(vd);
            telemetry.update();
        }

        vision.shutdown();
    }
}
