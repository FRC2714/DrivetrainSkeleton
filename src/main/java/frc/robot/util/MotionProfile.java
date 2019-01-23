package frc.robot.util;

public class MotionProfile {

    double angle, velocity, x, y;

    public MotionProfile(double angle, double velocity, double x, double y) {
        this.angle = angle;
        this.velocity = velocity;
        this.x = x;
        this.y = y;
    }

    public double getOrthogonalDisplacement(double currentX, double currentY) {
        double errorY = currentY - y;
        double errorX = currentX - x;
        double unitX = Math.cos(Math.toRadians(angle + 90));
        double unitY = Math.sin(Math.toRadians(angle + 90));
        double dotProduct = (unitX * errorX) + (unitY * errorY);

        return dotProduct;
    }

    public double getTangentialDisplacement(double currentX, double currentY) {
        double errorY = currentY - y;
        double errorX = currentX - x;
        double unitX = Math.cos(Math.toRadians(angle));
        double unitY = Math.sin(Math.toRadians(angle));
        double dotProduct = (unitX * errorX) + (unitY * errorY);

        return dotProduct;
    }

    // Calculate distance
    public double distanceCalc(double x1, double x2, double y1, double y2) {
        return Math.pow((Math.pow(x2 - x1, 2)) + (Math.pow(y2 - y1, 2)), 0.5);
    }
}