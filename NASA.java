package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "controlat", group = "gruoic")
public class NASA extends LinearOpMode {

    DcMotor left_f;
    DcMotor left_b;
    DcMotor right_f;
    DcMotor right_b;
    DcMotor drill;
    DcMotor colectare;
    DcMotor glisi;



    public void DriveC()
{
    double r=Math.hypot(-gamepad1.right_stick_x,gamepad1.right_stick_y);
    double angle=Math.atan2(gamepad1.right_stick_y,-gamepad1.right_stick_x);
    double vs,vd;

    vd=Math.sin(angle-Math.PI/4)*r;
    vs=Math.cos(angle-Math.PI/4)*r;

    left_b.setPower(vs);
    left_f.setPower(vs);
    right_b.setPower(-vd);
    right_f.setPower(-vd);
}

public void Drilling()
{
    if(gamepad1.b)
    {
        drill.setPower(-1);
    }
    else
        drill.setPower(0);
}

public void glisi()
{
    if(gamepad1.dpad_up)
    {
        glisi.setPower(1);
    }
    else
    {
        if(gamepad1.dpad_down)
            glisi.setPower(-1);
        else
            glisi.setPower(0);
    }
}

public void colectare()
{
    if(gamepad1.dpad_right)
    {
        colectare.setPower(1);
    }
    else
    {
        if(gamepad1.dpad_left)
            colectare.setPower(-1);
        else
            colectare.setPower(0);
    }
}
    @Override
    public void runOpMode() throws InterruptedException {

        left_b=hardwareMap.get(DcMotor.class,"left_b");
        left_f=hardwareMap.get(DcMotor.class,"left_f");
        right_b=hardwareMap.get(DcMotor.class,"right_b");
        right_f=hardwareMap.get(DcMotor.class,"right_f");
        drill=hardwareMap.get(DcMotor.class, "drill");
        colectare=hardwareMap.get(DcMotor.class, "colectare");
        glisi=hardwareMap.get(DcMotor.class, "glisi");


        right_b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_b.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
left_b.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
left_b.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();


        while (opModeIsActive() && !isStopRequested()) {

            DriveC();
            Drilling();
            glisi();
            colectare();
            telemetry.addData("",left_b.getCurrentPosition());
            telemetry.update();

        }

    }
}
