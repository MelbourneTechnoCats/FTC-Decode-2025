public class Ballistics {

        public static final double GRAVITY = 9.81;

        public static ShooterSolution solve(
                        double deltaX,
                        double deltaZ,
                        double robotVelocity,
                        double flightTime,
                        double velocityToRpmFactor,
                        double minAngleDeg,
                        double maxAngleDeg,
                        double maxLauncherVelocity) {

                if (flightTime <= 0) {
                        throw new IllegalArgumentException("Flight time must be positive");
                }

                // Required field-relative velocity
                double vX = deltaX / flightTime;
                double vZ = (deltaZ + 0.5 * GRAVITY * flightTime * flightTime) / flightTime;

                // Moving robot compensation
                double vLx = vX - robotVelocity;
                double vLz = vZ;

                // Impossible shot:
                // robot is moving faster toward target than the
                // required projectile horizontal speed.
                if (vLx <= 0) {
                        return ShooterSolution.invalid("Launcher would need to shoot backwards");
                }

                double launcherVelocity = Math.hypot(vLx, vLz);
                double angleDeg = Math.toDegrees(Math.atan2(vLz, vLx));

                if (angleDeg < minAngleDeg || angleDeg > maxAngleDeg) {
                        return ShooterSolution.invalid(
                                        String.format(
                                                        "Angle %.1f° outside limits [%.1f°, %.1f°]",
                                                        angleDeg,
                                                        minAngleDeg,
                                                        maxAngleDeg));
                }

                if (launcherVelocity > maxLauncherVelocity) {
                        return ShooterSolution.invalid(
                                        String.format(
                                                        "Velocity %.2f m/s exceeds max %.2f m/s",
                                                        launcherVelocity,
                                                        maxLauncherVelocity));
                }

                double rpm = launcherVelocity * velocityToRpmFactor;

                return ShooterSolution.valid(launcherVelocity, rpm, angleDeg, flightTime);
        }

        public static class ShooterSolution {

                public final boolean valid;
                public final double launcherVelocity;
                public final double rpm;
                public final double hoodAngleDegrees;
                public final double flightTime;
                public final String failureReason;

                private ShooterSolution(
                                boolean valid,
                                double launcherVelocity,
                                double rpm,
                                double hoodAngleDegrees,
                                double flightTime,
                                String failureReason) {

                        this.valid = valid;
                        this.launcherVelocity = launcherVelocity;
                        this.rpm = rpm;
                        this.hoodAngleDegrees = hoodAngleDegrees;
                        this.flightTime = flightTime;
                        this.failureReason = failureReason;
                }

                public static ShooterSolution valid(
                                double velocity,
                                double rpm,
                                double angleDeg,
                                double flightTime) {

                        return new ShooterSolution(true, velocity, rpm, angleDeg, flightTime, null);
                }

                public static ShooterSolution invalid(String reason) {

                        return new ShooterSolution(false, 0, 0, 0, 0, reason);
                }
        }
}