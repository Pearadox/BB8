
package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="BB8")
//@Disabled

public class BB8 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_motor = null;
    private DcMotor left_motor = null;
    private DcMotor right_motor = null;
    @Override
    public void init() {
        front_motor = hardwareMap.get(DcMotor.class, "front_motor");
        left_motor = hardwareMap.get(DcMotor.class,"left_motor");
        right_motor = hardwareMap.get(DcMotor.class,"right_motor");
        front_motor.setDirection(DcMotor.Direction.FORWARD);
        left_motor.setDirection(DcMotor.Direction.FORWARD);
        right_motor.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        front_motor.setPower(0);
        left_motor.setPower(1.0);
        right_motor.setPower(-1.00);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {


    }

}
