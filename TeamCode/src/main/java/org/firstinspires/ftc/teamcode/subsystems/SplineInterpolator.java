package org.firstinspires.ftc.teamcode.subsystems;

/**
 * Natural Cubic Spline Interpolator (C² continuous).
 * 
 * Constructs a smooth cubic spline from strictly increasing x values.
 * Excellent for shooter distance-to-velocity/hood tables.
 */
public class SplineInterpolator {

    private final int n;
    private final double[] x;
    private final double[] y;
    private final double[] a;     // coefficients for cubic term
    private final double[] b;
    private final double[] c;
    private final double[] d;

    /**
     * Build a natural cubic spline from sample points.
     *
     * @param x sample x values (distances), must be strictly increasing
     * @param y sample y values (velocity or hood ticks)
     */
    public SplineInterpolator(double[] x, double[] y) {
        if (x == null || y == null || x.length != y.length || x.length < 2) {
            throw new IllegalArgumentException("Need at least 2 points with matching x/y arrays");
        }

        this.n = x.length;
        this.x = x.clone();
        this.y = y.clone();

        this.a = new double[n];
        this.b = new double[n];
        this.c = new double[n];
        this.d = new double[n];

        buildSpline();
    }

    private void buildSpline() {
        // Step 1: Copy y into a (we solve for the rest)
        System.arraycopy(y, 0, a, 0, n);

        double[] h = new double[n - 1];
        for (int i = 0; i < n - 1; i++) {
            h[i] = x[i + 1] - x[i];
            if (h[i] <= 0) {
                throw new IllegalArgumentException("x values must be strictly increasing");
            }
        }

        // Natural cubic spline: second derivatives at endpoints = 0
        double[] alpha = new double[n - 1];
        for (int i = 1; i < n - 1; i++) {
            alpha[i] = (3.0 / h[i]) * (a[i + 1] - a[i]) 
                     - (3.0 / h[i - 1]) * (a[i] - a[i - 1]);
        }

        double[] l = new double[n];
        double[] mu = new double[n];
        double[] z = new double[n];

        l[0] = 1.0;
        mu[0] = 0.0;
        z[0] = 0.0;

        for (int i = 1; i < n - 1; i++) {
            l[i] = 2.0 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }

        l[n - 1] = 1.0;
        z[n - 1] = 0.0;
        c[n - 1] = 0.0;

        // Back substitution
        for (int j = n - 2; j >= 0; j--) {
            c[j] = z[j] - mu[j] * c[j + 1];
            b[j] = (a[j + 1] - a[j]) / h[j] - h[j] * (c[j + 1] + 2.0 * c[j]) / 3.0;
            d[j] = (c[j + 1] - c[j]) / (3.0 * h[j]);
        }
    }

    /**
     * Evaluate the cubic spline at xi.
     * Extrapolates linearly outside the range.
     */
    public double interpolate(double xi) {
        if (xi <= x[0]) {
            // Linear extrapolation using first segment
            double dx = xi - x[0];
            return y[0] + b[0] * dx;
        }
        if (xi >= x[n - 1]) {
            // Linear extrapolation using last segment
            double dx = xi - x[n - 2];
            return y[n - 1] + (b[n - 2] + 2.0 * c[n - 2] * (x[n - 1] - x[n - 2]) 
                             + 3.0 * d[n - 2] * Math.pow(x[n - 1] - x[n - 2], 2)) * dx;
        }

        // Binary search to find the right interval
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

        double dx = xi - x[low];
        return a[low] + b[low] * dx + c[low] * dx * dx + d[low] * dx * dx * dx;
    }
}