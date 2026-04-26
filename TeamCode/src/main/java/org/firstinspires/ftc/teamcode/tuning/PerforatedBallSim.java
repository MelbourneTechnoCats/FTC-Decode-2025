//import javax.swing.*;
//import java.awt.*;
//import java.util.ArrayList;
//import java.util.List;
//
//public class PerforatedBallSim extends JPanel {
//    // Physical Constants
//    private final double G = 9.81;
//    private final double RHO = 1.225; // Air density
//    private final double CD = 0.6;    // Drag coefficient for perforated ball
//    private final double MASS = 0.05; // 50 grams
//    private final double AREA = Math.PI * Math.pow(0.125 / 2.0, 2); // 125mm diameter
//    private final double K = (0.5 * RHO * CD * AREA) / MASS;
//
//    private List<Point2D> airPath;
//    private List<Point2D> vacuumPath;
//
//    public PerforatedBallSim(double vKmh, double angleDeg) {
//        double v0 = vKmh / 3.6;
//        double angleRad = Math.toRadians(angleDeg);
//
//        airPath = calculateRK4(v0, angleRad);
//        vacuumPath = calculateVacuum(v0, angleRad);
//    }
//
//    // RK4 ODE Solver for path with air resistance
//    private List<Point2D> calculateRK4(double v0, double angle) {
//        List<Point2D> path = new ArrayList<>();
//        double dt = 0.01;
//        double[] state = {0, 0, v0 * Math.cos(angle), v0 * Math.sin(angle)}; // x, y, vx, vy
//
//        while (state[1] >= 0) {
//            path.add(new Point2D(state[0], state[1]));
//
//            double[] k1 = derivatives(state);
//            double[] k2 = derivatives(step(state, k1, dt / 2));
//            double[] k3 = derivatives(step(state, k2, dt / 2));
//            double[] k4 = derivatives(step(state, k3, dt));
//
//            for (int i = 0; i < 4; i++) {
//                state[i] += (dt / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
//            }
//        }
//        return path;
//    }
//
//    private double[] derivatives(double[] s) {
//        double vx = s[2], vy = s[3];
//        double v = Math.sqrt(vx * vx + vy * vy);
//        return new double[]{vx, vy, -K * v * vx, -G - (K * v * vy)};
//    }
//
//    private double[] step(double[] s, double[] k, double dt) {
//        double[] res = new double[4];
//        for (int i = 0; i < 4; i++) res[i] = s[i] + k[i] * dt;
//        return res;
//    }
//
//    private List<Point2D> calculateVacuum(double v0, double angle) {
//        List<Point2D> path = new ArrayList<>();
//        double vx = v0 * Math.cos(angle);
//        double vy = v0 * Math.sin(angle);
//        for (double t = 0; ; t += 0.01) {
//            double x = vx * t;
//            double y = vy * t - 0.5 * G * t * t;
//            if (y < 0 && t > 0) break;
//            path.add(new Point2D(x, y));
//        }
//        return path;
//    }
//
//    @Override
//    protected void paintComponent(Graphics g) {
//        super.paintComponent(g);
//        Graphics2D g2 = (Graphics2D) g;
//        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
//
//        int scale = 25; // Pixels per meter
//        int offsetX = 50;
//        int offsetY = getHeight() - 50;
//
//        // Draw Vacuum Path (Blue dashed)
//        g2.setColor(Color.BLUE);
//        drawPath(g2, vacuumPath, scale, offsetX, offsetY);
//
//        // Draw Air Path (Red solid)
//        g2.setColor(Color.RED);
//        drawPath(g2, airPath, scale, offsetX, offsetY);
//
//        g2.setColor(Color.BLACK);
//        g2.drawString("Red: Air Resistance (RK4) | Blue: Vacuum", 50, 30);
//    }
//
//    private void drawPath(Graphics2D g2, List<Point2D> path, int scale, int ox, int oy) {
//        for (int i = 0; i < path.size() - 1; i++) {
//            g2.drawLine(ox + (int)(path.get(i).x * scale), oy - (int)(path.get(i).y * scale),
//                        ox + (int)(path.get(i+1).x * scale), oy - (int)(path.get(i+1).y * scale));
//        }
//    }
//
//    record Point2D(double x, double y) {}
//
//    public static void main(String[] args) {
//        JFrame frame = new JFrame("Perforated Ball Trajectory - RK4");
//        frame.add(new Perfor