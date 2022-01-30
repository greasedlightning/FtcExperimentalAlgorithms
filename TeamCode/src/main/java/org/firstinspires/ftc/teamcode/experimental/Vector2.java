package org.firstinspires.ftc.teamcode.experimental;

public class Vector2 {
    public int x;
    public int y;
    public int c;

    public Vector2(int _x, int _y) {
        x = _x;
        y = _y;
        c = 0;
    }

    public void clean() {
        if (c != 0) {
            x /= c;
            y /= c;
        }
    }

    public double distTo(Vector2 v) {
        return Math.sqrt((x - v.x)*(x - v.x) + (y - v.y)*(y - v.y));
    }
    public double angleTo(Vector2 v) {
        double angle = Math.atan2(y - v.y, x - v.x);
        return angle*180/Math.PI;
    }
}