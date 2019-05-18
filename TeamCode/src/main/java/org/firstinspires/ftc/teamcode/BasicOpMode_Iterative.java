
package org.firstinspires.ftc.teamcode;

import android.media.AudioManager;
import android.media.MediaPlayer;
import android.net.Uri;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Test stuff", group="Iterative Opmode")
@Disabled

public class BasicOpMode_Iterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    MediaPlayer mediaPlayer;

    @Override
    public void init() {
        telemetry.addData("Status", "Hoop de doop");

        mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.yoohoo);
    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Yes sirreee");
    }

    @Override
    public void start() {
        runtime.reset();
        mediaPlayer.start();
    }

    @Override
    public void loop() {
        telemetry.addData("Statussy", "Run Blah Time: " + runtime.toString());

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        telemetry.addData("Stop", "Stop");
    }

}
