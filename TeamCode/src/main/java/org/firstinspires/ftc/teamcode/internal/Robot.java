package org.firstinspires.ftc.teamcode.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static org.firstinspires.ftc.teamcode.internal.Robot.RobotDriveType.MECANUM;
import static org.firstinspires.ftc.teamcode.internal.Robot.RobotDriveType.STANDARD;

public class Robot {
    public double drivePower = 1;
    private static final double INCHES_PER_ROTATION = 3.95 * Math.PI;
    private static final double TICKS_PER_INCH = 537.6 / INCHES_PER_ROTATION;

    private final OpMode opMode;

    public enum RobotDriveType {
        @SuppressWarnings("unused") STANDARD,
        @SuppressWarnings("SpellCheckingInspection") MECANUM
    }

    private BNO055IMU imu;

    private final RobotDriveType driveType = STANDARD;

    private DcMotor driveLeftFront;
    private DcMotor driveRightFront;
    private DcMotor driveLeftRear;
    private DcMotor driveRightRear;

    private DcMotor gate;

    public Orientation orientation = new Orientation();

    public String error;

    public Robot(OpMode opMode) {
        this.opMode = opMode;
    }

    public void init() {
        HardwareMap hardwareMap = opMode.hardwareMap;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        drivePower = 0.5;

        driveLeftFront = hardwareMap.get(DcMotor.class, "driveLeftFront");
        driveLeftFront.setDirection(REVERSE);
        driveLeftFront.setZeroPowerBehavior(BRAKE);
        driveLeftFront.setMode(STOP_AND_RESET_ENCODER);
        driveLeftFront.setMode(RUN_USING_ENCODER);

        driveRightFront = hardwareMap.get(DcMotor.class,"driveRightFront");
        driveRightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        driveRightFront.setZeroPowerBehavior(BRAKE);
        driveRightFront.setMode(STOP_AND_RESET_ENCODER);
        driveRightFront.setMode(RUN_USING_ENCODER);

        driveLeftRear = hardwareMap.get(DcMotor.class,"driveLeftRear");
        driveLeftRear.setDirection(REVERSE);
        driveLeftRear.setZeroPowerBehavior(BRAKE);
        driveLeftRear.setMode(STOP_AND_RESET_ENCODER);
        driveLeftRear.setMode(RUN_USING_ENCODER);

        driveRightRear = hardwareMap.get(DcMotor.class, "driveRightRear");
        driveRightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        driveRightRear.setZeroPowerBehavior(BRAKE);
        driveRightRear.setMode(STOP_AND_RESET_ENCODER);
        driveRightRear.setMode(RUN_USING_ENCODER);

        gate = hardwareMap.get(DcMotor.class, "gate");
        gate.setDirection(REVERSE);
        gate.setZeroPowerBehavior(BRAKE);
        gate.setMode(STOP_AND_RESET_ENCODER);
        gate.setMode(RUN_USING_ENCODER);
    }

    public void calibrate() {

    }

    public void start() {

    }

    public void drive(double drive, double strafe, double turn) {
        if (opMode.isStopping()) return;

        if (driveType != MECANUM) strafe = 0;

        // Since left stick can be pushed in all directions to control the robot's movements, its "power" must be the actual
        // distance from the center, or the hypotenuse of the right triangle formed by left_stick_x and left_stick_y
        double r = Math.hypot(strafe, drive);

        // Angle between x axis and "coordinates" of left stick
        double robotAngle = Math.atan2(drive, strafe) - Math.PI / 4;

        double lf = drivePower * (r * Math.cos(robotAngle) + turn);
        double lr = drivePower * (r * Math.sin(robotAngle) + turn);
        double rf = drivePower * (r * Math.sin(robotAngle) - turn);
        double rr = drivePower * (r * Math.cos(robotAngle) - turn);

        driveLeftFront.setPower(lf);
        driveRightFront.setPower(rf);
        driveLeftRear.setPower(lr);
        driveRightRear.setPower(rr);
    }

    public void driveInches(double drive, double strafe, double heading, double inches) {
        if (opMode.isStopping()) return;

        double power = clamp(0.2, 1.0, drive + strafe);

        turn(power, heading);

        resetEncoders();

        int targetPosition = (int)(inches * TICKS_PER_INCH);
        int position = 0;

        double remainder, turn;

        while (!opMode.isStopping() && targetPosition - position > 0) {
            remainder = getRemainderLeftToTurn(heading);
            if (drive != 0) drive = clamp(0.2, drive, (targetPosition - position) / (TICKS_PER_INCH * 12));
            if (strafe != 0) strafe = clamp(0.2, strafe, (targetPosition - position) / (TICKS_PER_INCH * 12));
            turn = remainder / 45;
            drive(drive, strafe, turn);

            position = (
                Math.abs(driveLeftFront.getCurrentPosition()) +
                    Math.abs(driveLeftRear.getCurrentPosition()) +
                    Math.abs(driveRightFront.getCurrentPosition()) +
                    Math.abs(driveRightRear.getCurrentPosition())
            ) / 4;
        }

        this.drive(0, 0, 0);
    }

    public void driveSeconds(double drive, double strafe, double heading, double seconds) {
        if (opMode.isStopping()) return;

        double power = clamp(0.2, 1.0, drive + strafe);

        turn(power, heading);

        resetEncoders();

        double targetTime = seconds * 1_000_000_000;
        double currentTime = System.nanoTime();

        double remainder, turn;

        while (!opMode.isStopping() && targetTime - currentTime > 0) {
            remainder = getRemainderLeftToTurn(heading);
            if (drive != 0) drive = clamp(0.2, drive, (targetTime - currentTime) / 500_000_000);
            if (strafe != 0) strafe = clamp(0.2, strafe, (targetTime - currentTime) / 500_000_000);
            turn = remainder / 45;
            drive(drive, strafe, turn);
            currentTime = System.nanoTime();
        }

        this.drive(0, 0, 0);
    }

    public void turn(double power, double heading) {
        if (opMode.isStopping()) return;

        power = Math.abs(power);

        double remainder, turn;

        do {
            remainder = getRemainderLeftToTurn(heading);
            turn = clamp(0.2, power, remainder / 45 * power);
            drive(0, 0,turn);
        } while (!opMode.isStopping() && (remainder < -1 || remainder > 1));

        drive(0, 0, 0);
    }

    public void moveGate(double power) {
        gate.setPower(power);
    }

    public void addTelemetry() {
        Telemetry telemetry = opMode.telemetry;

        telemetry.addData("Drive", "%.2f Pow", opMode.gamepad1.left_stick_y);
        telemetry.addData("Turn", "%.2f Pow", opMode.gamepad1.right_stick_x);
        telemetry.addData("Drive (LF)", "%.2f Pow, %d Pos", driveLeftFront.getPower(), driveLeftFront.getCurrentPosition());
        telemetry.addData("Drive (LR)", "%.2f Pow, %d Pos", driveLeftRear.getPower(), driveLeftRear.getCurrentPosition());
        telemetry.addData("Drive (RF)", "%.2f Pow, %d Pos", driveRightFront.getPower(), driveRightFront.getCurrentPosition());
        telemetry.addData("Drive (RR)", "%.2f Pow, %d Pos", driveRightRear.getPower(), driveRightRear.getCurrentPosition());
        telemetry.addData("Gate", "%.2f Pow, %d Pos", gate.getPower(), gate.getCurrentPosition());

        telemetry.addLine();

        if (error != null && !error.isEmpty())
            telemetry.addData("Error", error);
    }

    public Orientation getOrientation() {
        return imu.getAngularOrientation(INTRINSIC, ZYX, DEGREES);
    }

    private double getRemainderLeftToTurn(double heading) {
        double remainder;
        orientation = getOrientation();
        remainder = orientation.firstAngle - heading;
        if (remainder > +180) remainder -= 360;
        if (remainder < -180) remainder += 360;
        return remainder;
    }

    private void resetEncoders() {
        driveLeftFront.setMode(STOP_AND_RESET_ENCODER);
        driveLeftFront.setMode(RUN_USING_ENCODER);
        driveLeftRear.setMode(STOP_AND_RESET_ENCODER);
        driveLeftRear.setMode(RUN_USING_ENCODER);
        driveRightFront.setMode(STOP_AND_RESET_ENCODER);
        driveRightFront.setMode(RUN_USING_ENCODER);
        driveRightRear.setMode(STOP_AND_RESET_ENCODER);
        driveRightRear.setMode(RUN_USING_ENCODER);
    }

    private double clamp(double min, double max, double value) {
        return value >= 0 ?
            Math.min(max, Math.max(min, value)) :
            Math.min(-min, Math.max(-max, value));
    }
}