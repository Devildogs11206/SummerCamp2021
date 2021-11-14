package org.firstinspires.ftc.teamcode.internal;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.opmodes.OpMode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static com.qualcomm.robotcore.hardware.DigitalChannel.Mode.INPUT;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.ZYX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.INTRINSIC;
import static org.firstinspires.ftc.teamcode.internal.Robot.RobotDriveType.MECANUM;

public class Robot {
    public double drivePower = 1;
    private static final double INCHES_PER_ROTATION = 3.95 * Math.PI;
    private static final double TICKS_PER_INCH = 537.6 / INCHES_PER_ROTATION;

    private final OpMode opMode;

    private BNO055IMU imu;

    public enum RobotDriveType {
        @SuppressWarnings("unused") STANDARD, MECANUM
    }

    private final RobotDriveType driveType = MECANUM;

    private DcMotor driveLeftFront;
    private DcMotor driveRightFront;
    private DcMotor driveLeftRear;
    private DcMotor driveRightRear;

    private DcMotor lift;

    private DigitalChannel liftLimitLeftFront;
    private DigitalChannel liftLimitRightFront;

    private DcMotor intake;

    public boolean navigationTargetVisible = false;
    public Position position = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
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
        driveRightFront.setDirection(FORWARD);
        driveRightFront.setZeroPowerBehavior(BRAKE);
        driveRightFront.setMode(STOP_AND_RESET_ENCODER);
        driveRightFront.setMode(RUN_USING_ENCODER);

        driveLeftRear = hardwareMap.get(DcMotor.class,"driveLeftRear");
        driveLeftRear.setDirection(REVERSE);
        driveLeftRear.setZeroPowerBehavior(BRAKE);
        driveLeftRear.setMode(STOP_AND_RESET_ENCODER);
        driveLeftRear.setMode(RUN_USING_ENCODER);

        driveRightRear = hardwareMap.get(DcMotor.class, "driveRightRear");
        driveRightRear.setDirection(FORWARD);
        driveRightRear.setZeroPowerBehavior(BRAKE);
        driveRightRear.setMode(STOP_AND_RESET_ENCODER);
        driveRightRear.setMode(RUN_USING_ENCODER);

        lift = hardwareMap.get(DcMotor.class, "lift");
        lift.setDirection(FORWARD);
        lift.setZeroPowerBehavior(BRAKE);
        lift.setMode(STOP_AND_RESET_ENCODER);
        lift.setMode(RUN_USING_ENCODER);

        liftLimitLeftFront = hardwareMap.get(DigitalChannel.class, "liftLimitLeftFront");
        liftLimitLeftFront.setMode(INPUT);
        liftLimitRightFront = hardwareMap.get(DigitalChannel.class, "liftLimitRightFront");
        liftLimitRightFront.setMode(INPUT);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(FORWARD);
        intake.setZeroPowerBehavior(BRAKE);
        intake.setMode(STOP_AND_RESET_ENCODER);
        intake.setMode(RUN_USING_ENCODER);
    }

    public void calibrate() {

    }

    public void start() {
        lift(LiftMode.FORWARD);

        while (!this.isLiftAtFrontLimit())
            opMode.sleep(50);

        lift(LiftMode.STOPPED);

        opMode.sleep(50);

        lift.setMode(STOP_AND_RESET_ENCODER);
        lift.setMode(RUN_USING_ENCODER);
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

    public void drive(double drive, double strafe, double heading, double inches) {
        if (opMode.isStopping()) return;

        double power = clamp(0.2, 1.0, drive + strafe);

        turn(power, heading);

        resetEncoders();

        int targetPosition = (int) (inches * TICKS_PER_INCH);
        int position = 0;

        double remainder, turn;

        while (!opMode.isStopping() && targetPosition - position > 0) {
            remainder = getRemainderLeftToTurn(heading);
            if (drive != 0)
                drive = clamp(0.2, drive, (targetPosition - position) / (TICKS_PER_INCH * 12));
            if (strafe != 0)
                strafe = clamp(0.2, strafe, (targetPosition - position) / (TICKS_PER_INCH * 12));
            turn = remainder / 45;
            drive(drive, strafe, turn);

            position = (
                Math.abs(driveLeftFront.getCurrentPosition()) +
                Math.abs(driveLeftRear.getCurrentPosition()) +
                Math.abs(driveRightFront.getCurrentPosition()) +
                Math.abs(driveRightRear.getCurrentPosition())
            ) / 4;
        }
    }

    public void driveTest() {
        driveLeftFront.setPower(drivePower);
        driveRightFront.setPower(drivePower);
        driveLeftRear.setPower(drivePower);
        driveRightRear.setPower(drivePower);

        opMode.sleep(10000);

        driveLeftFront.setPower(0);
        driveRightFront.setPower(0);
        driveLeftRear.setPower(0);
        driveRightRear.setPower(0);
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

    public enum LiftMode {
        STOPPED(0), FORWARD(1.00), BACKWARD(-1.00);

        public double power;

        LiftMode(double power) {
            this.power = power;
        }
    }

    public void lift(LiftMode mode) {
        if ((mode == LiftMode.FORWARD && this.isLiftAtFrontLimit()) ||
            (mode == LiftMode.BACKWARD && lift.getCurrentPosition() < LiftPosition.MAX.position))
            mode = LiftMode.STOPPED;

        lift.setMode(RUN_USING_ENCODER);
        lift.setPower(mode.power);
    }

    public void lift(LiftMode mode, int milliseconds) {
        lift(mode);
        opMode.sleep(milliseconds);
        lift(LiftMode.STOPPED);
    }

    public enum LiftPosition {
        FORWARD(0), LOWGOAL(-1250), MIDGOAL(-2900), CAROUSEL(-2870), MAX(-4000);

        public int position;

        LiftPosition(int position) {
            this.position = position;
        }
    }

    public void lift(LiftPosition position) {
        lift.setTargetPosition(position.position);
        lift.setMode(RUN_TO_POSITION);
        lift.setPower(LiftMode.FORWARD.power);
    }

    public enum IntakeMode {
        FORWARD(0.50), NEUTRAL(0), REVERSE(-0.50), CAROUSEL(0.15), REVERSECAROUSEL(-0.15);

        public double power;

        IntakeMode(double power) {
            this.power = power;
        }
    }

    public void intake(IntakeMode mode) {
        intake.setPower(mode.power);
    }

    public void intake(IntakeMode mode, int milliseconds) {
        intake(mode);
        opMode.sleep(milliseconds);
        intake(IntakeMode.NEUTRAL);
    }

    public void addTelemetry() {
        Telemetry telemetry = opMode.telemetry;

        telemetry.addData("Drive", "%.2f Pow", opMode.gamepad1.left_stick_y);
        telemetry.addData("Turn", "%.2f Pow", opMode.gamepad1.right_stick_x);
        telemetry.addData("Drive (LF)", "%.2f Pow, %d Pos", driveLeftFront.getPower(), driveLeftFront.getCurrentPosition());
        telemetry.addData("Drive (RF)", "%.2f Pow, %d Pos", driveRightFront.getPower(), driveRightFront.getCurrentPosition());
        telemetry.addData("Drive (LR)", "%.2f Pow, %d Pos", driveLeftRear.getPower(), driveLeftRear.getCurrentPosition());
        telemetry.addData("Drive (RR)", "%.2f Pow, %d Pos", driveRightRear.getPower(), driveRightRear.getCurrentPosition());

        telemetry.addData("Lift", "%.2f Pow, %d Pos", lift.getPower(), lift.getCurrentPosition());
        telemetry.addData("Intake", "%.2f Pow, %d Pos", intake.getPower(), intake.getCurrentPosition());
        telemetry.addData("Lift Limit Left Front", liftLimitLeftFront.getState());
        telemetry.addData("Lift Limit Right Front", liftLimitRightFront.getState());

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

    private boolean isLiftAtFrontLimit() {
        boolean leftPressed = !liftLimitLeftFront.getState();
        boolean rightPressed = !liftLimitRightFront.getState();
        return leftPressed || rightPressed;
    }
}