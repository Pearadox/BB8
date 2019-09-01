
package org.firstinspires.ftc.teamcode;

import android.media.MediaPlayer;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@TeleOp(name="BB8")
//@Disabled

public class BB8 extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();//sets up a timer in the program
    private DcMotor front_motor = null;
    private DcMotor left_motor = null;
    private DcMotor right_motor = null;

    MediaPlayer mediaPlayer;

    Servo leftServo;
    Servo rightServo;

    final double SERVO_MAX_ANGLE = 201;

    BNO055IMU imu;
    Orientation lastAngles;

    double headingOffset = 0;
    double lastvoltrun = -1;
    double globalAngle = 0;

    boolean lastABtn = false;
    boolean lastXBtn = false;
    boolean lastYBtn = false;

    @Override
    public void init() {
        front_motor = hardwareMap.get(DcMotor.class, "front_motor");
        left_motor = hardwareMap.get(DcMotor.class,"left_motor");
        right_motor = hardwareMap.get(DcMotor.class,"right_motor");
        front_motor.setDirection(DcMotor.Direction.FORWARD);
        left_motor.setDirection(DcMotor.Direction.FORWARD);
        right_motor.setDirection(DcMotor.Direction.FORWARD);

        mediaPlayer = MediaPlayer.create(hardwareMap.appContext, R.raw.giddy);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        leftServo = hardwareMap.servo.get("leftServo");
        rightServo = hardwareMap.servo.get("rightServo");
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
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

    }

    @Override
    public void loop() {
        double voltage = getBatteryVoltage();
        double heading = getAngle();
        double lastError = 0;

        if(voltage>10.5) {
            lastvoltrun = runtime.milliseconds();
        }
        else if(runtime.milliseconds() - lastvoltrun > 1000){
            int[] errorrrrr = new int[1];
            int awfes = errorrrrr[5414];
        }

        if(gamepad1.a && !lastABtn) {
            mediaPlayer.start();
        }
        lastABtn = gamepad1.a;

        if(gamepad1.x && !lastXBtn) {
            headingOffset -= 10;
        }
        lastXBtn = gamepad1.x;

        if(gamepad1.y && !lastYBtn) {
            headingOffset += 10;
        }
        lastYBtn = gamepad1.y;

        double input = 0;
        double setpoint = heading;

        double error = input - setpoint;

        double kP = 0.02;

        double P = kP * error;

        double output = P;
        lastError = error;

        output = 0;
        output = (gamepad1.right_stick_x);

        double front = output;
        double left = output;
        double right = output;

        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;

        heading += headingOffset;

        front += x*Math.cos(Math.toRadians(heading)) - y*Math.sin(Math.toRadians(heading));
        left += x*Math.cos(Math.toRadians(heading + 120)) - y*Math.sin(Math.toRadians(heading + 120));
        right += x*Math.cos(Math.toRadians(heading + 240)) - y*Math.sin(Math.toRadians(heading + 240));

//        front += x;
//        left += 0.5 * x - Math.sqrt(3)/2 * y;
//        right += 0.5 * x + Math.sqrt(3)/2 * y;
//
        front_motor.setPower(front);
        left_motor.setPower(left);
        right_motor.setPower(right);

        double righty = -gamepad1.right_stick_y;
        double rightx = -gamepad1.right_stick_x;

        double thetaMax = 90;
        double thetaY = -thetaMax/2. * righty + thetaMax/2.;
        double thetaXRight = thetaMax/2. * rightx;
        double thetaXLeft = -thetaMax/2. * rightx;

        double angleRight = thetaY + thetaXRight;
        double angleLeft = thetaY + thetaXLeft;

        leftServo.setPosition(angleLeft / SERVO_MAX_ANGLE);
        rightServo.setPosition(angleRight / SERVO_MAX_ANGLE);

        telemetry.addData("LeftOutput", angleLeft);
        telemetry.addData("RightOutput", angleRight);

//        telemetry.addData("joystick left y", gamepad1.left_stick_y);
//         telemetry.addData("joystick right y", gamepad2.right_stick_y);
        telemetry.addData("heading", heading);
        telemetry.addData("P", P);
        telemetry.addData("Output", output);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {


    }

    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }

    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if(deltaAngle < -180) {
            deltaAngle += 360;
        }
        else if(deltaAngle > 180) {
            deltaAngle -= 360;
        }

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
}
