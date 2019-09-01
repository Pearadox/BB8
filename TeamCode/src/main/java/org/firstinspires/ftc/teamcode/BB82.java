
package org.firstinspires.ftc.teamcode;

import android.media.AudioManager;
import android.media.MediaPlayer;
import android.net.Uri;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Test", group="Iterative Opmode")
//@Disabled

public class BB82 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    MediaPlayer mediaPlayer;
    Servo leftServo;
    Servo rightServo;
    Servo rotationServo;

    final double SERVO_MAX_ANGLE = 201;

    @Override
    public void init() {
        leftServo = hardwareMap.servo.get("leftServo");
        rightServo = hardwareMap.servo.get("rightServo");
        rotationServo = hardwareMap.servo.get("rotationServo");
        rightServo.setDirection(Servo.Direction.REVERSE);

        leftServo.scaleRange(0, SERVO_MAX_ANGLE);
        rightServo.scaleRange(0, SERVO_MAX_ANGLE);
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
        double righty = -gamepad1.right_stick_y;
        double rightx = -gamepad1.right_stick_x;
        boolean trigL = gamepad1.left_trigger > 0.5;
        boolean trigR = gamepad1.right_trigger > 0.5;
        boolean buttonA = gamepad1.a;
        boolean buttonB = gamepad1.b;
        boolean buttonX = gamepad1.x;
        boolean buttonY = gamepad1.y;

        double thetaMax = 90;
        double thetaY = -thetaMax/2. * righty + thetaMax/2.;
        double thetaXRight = thetaMax/2. * rightx;
        double thetaXLeft = -thetaMax/2. * rightx;

        double angleRight = thetaY + thetaXRight;
        double angleLeft = thetaY + thetaXLeft;

        leftServo.setPosition(angleLeft / SERVO_MAX_ANGLE);
        rightServo.setPosition(angleRight / SERVO_MAX_ANGLE);

        telemetry.addData("ThetaY", thetaY);
        telemetry.addData("ThetaX", thetaXRight);

        telemetry.addData("LeftOutput", angleLeft);
        telemetry.addData("LeftOutput2", angleLeft/SERVO_MAX_ANGLE);

        telemetry.addData("RightOutput", angleRight);
        telemetry.addData("RightOutput2", angleRight/SERVO_MAX_ANGLE);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Stop", "Stop");
    }

}

