package org.firstinspires.ftc.teamcode.code.Robot.Archive;

/**
 * Drivetrain math for a four-wheel mecanum (or standard tank) chassis.
 *
 * <p>These methods only compute the four wheel powers — they do NOT touch
 * hardware — so they are easy to unit test and reuse between op modes.
 * Feed the results into your motors with {@link MotorUtil#setPowers}.
 *
 * <p>Wheel order everywhere in this class is:
 * front-left, front-right, back-left, back-right.
 */
public final class DriveUtil {

    private DriveUtil() {}

    /**
     * Field-agnostic robot-centric mecanum mixing.
     *
     * @param drive    forward/back (+ = forward), typically left stick Y (remember to negate gamepad Y)
     * @param strafe   left/right   (+ = right),   typically left stick X
     * @param rotate     rotation     (+ = clockwise), typically right stick X
     * @return array {frontLeft, frontRight, backLeft, backRight}, each scaled to [-1, 1]
     */
    public static double[] mecanum(double drive, double strafe, double rotate) {
        double frontLeft  = drive + strafe + rotate;
        double frontRight = drive - strafe - rotate;
        double backLeft   = drive - strafe + rotate;
        double backRight  = drive + strafe - rotate;
        return normalize(new double[]{frontLeft, frontRight, backLeft, backRight});
    }

    /**
     * Field-centric mecanum mixing. Uses the robot's current heading so pushing
     * the stick "away" always moves the robot away from the driver regardless of
     * which way the robot is facing.
     *
     * @param drive        forward/back relative to the driver (+ = away)
     * @param strafe       left/right relative to the driver (+ = right)
     * @param rotate         rotation (+ = clockwise)
     * @param headingRad   robot heading in radians (e.g. from the IMU)
     */
    public static double[] mecanumFieldCentric(double drive, double strafe,
                                               double rotate, double headingRad) {
        // Rotate the translation vector by -heading into the robot frame.
        double cos = Math.cos(-headingRad);
        double sin = Math.sin(-headingRad);
        double rotStrafe = strafe * cos - drive * sin;
        double rotDrive  = strafe * sin + drive * cos;
        return mecanum(rotDrive, rotStrafe, rotate);
    }

    /**
     * Standard arcade/tank mixing for a non-strafing 4-motor drive.
     *
     * @return array {frontLeft, frontRight, backLeft, backRight}
     */
    public static double[] arcade(double drive, double rotate) {
        double left  = drive + rotate;
        double right = drive - rotate;
        return normalize(new double[]{left, right, left, right});
    }

    /**
     * Scales all powers down proportionally if any magnitude exceeds 1.0, which
     * preserves the drive direction instead of clipping individual wheels.
     */
    public static double[] normalize(double[] powers) {
        double max = 0.0;
        for (double p : powers) {
            max = Math.max(max, Math.abs(p));
        }
        if (max > 1.0) {
            for (int i = 0; i < powers.length; i++) {
                powers[i] /= max;
            }
        }
        return powers;
    }

    /**
     * Multiplies every power by a scalar, e.g. a "slow mode" trigger of 0.4.
     */
    public static double[] scale(double[] powers, double factor) {
        for (int i = 0; i < powers.length; i++) {
            powers[i] *= factor;
        }
        return powers;
    }
}
