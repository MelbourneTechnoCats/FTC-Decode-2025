package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Natural cubic spline interpolator for monotonic increasing x values.
 * Construct with arrays of x (strictly increasing) and y of the same length.
 *
 * This class is intentionally *pure numeric*:
 *  - it represents a single 1D spline (x -> y)
 *  - it does not manage any lookup tables or per-angle maps
 *
 * Higher-level code (e.g. NewShooterSubsystem) is responsible for:
 *  - storing per-angle (distance, velocity) samples
 *  - building one SplineInterpolator per angle when the LUT changes
 */
public class SplineInterpolator {
    private final int n;
    private final double[] x;
    private final double[] a;
    private final double[] b;
    // `b` stores per-segment slopes for linear interpolation

    /**
     * Build a natural cubic spline from sample points.
     *
     * @param x sample x values, strictly increasing
     * @param y sample y values, same length as x
     */
    public SplineInterpolator(double[] x, double[] y) {
        if (x == null || y == null || x.length != y.length || x.length < 2) {
            throw new IllegalArgumentException("Need at least two points with matching x/y arrays");
        }
        this.n = x.length;
        this.x = x.clone();
        this.a = y.clone();
        this.b = new double[n - 1];
        // compute per-segment slopes and validate x monotonicity
        for (int i = 0; i < n - 1; i++) {
            double dx = x[i + 1] - x[i];
            if (dx <= 0.0) {
                throw new IllegalArgumentException("x must be strictly increasing");
            }
            b[i] = (a[i + 1] - a[i]) / dx;
        }
    }

    /**
     * Evaluate the spline at xi.
     * For xi outside the original x range, we extrapolate using the nearest segment.
     *
     * @param xi point to evaluate
     * @return interpolated value
     */
    public double interpolate(double xi) {
        // Handle out-of-bounds on the left by extrapolating with first segment
        int i;
        if (xi <= x[0]) {
            i = 0;
        // Handle out-of-bounds on the right by extrapolating with last segment
        } else if (xi >= x[n - 1]) {
            i = n - 2;
        } else {
            // Binary search for interval [x[low], x[low+1]] containing xi
            int low = 0;
            int high = n - 1;
            while (high - low > 1) {
                int mid = (low + high) >>> 1;
                if (x[mid] <= xi) {
                    low = mid;
                } else {
                    high = mid;
                }
            }
            i = low;
        }
        double dx = xi - x[i];
        return a[i] + b[i] * dx;
    }
}