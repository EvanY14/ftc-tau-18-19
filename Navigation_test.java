package org.firstinspires.ftc.teamcode;

import java.lang.reflect.Array;
import java.util.ArrayList;

public class Navigation_test extends AUTO_METHODS{

    public void runOpMode() throws InterruptedException{
        setUp(hardwareMap, telemetry);

        ArrayList<Double> location = new ArrayList<>();
        location.add(Math.sqrt(2) * 2 * 12);
        location.add(Math.sqrt(2) * 2 * 12);
        /*telemetry.addData("Robot x", getRobotX());
        telemetry.addData("Robot y", getRobotY());
        navigateTo(location);
        telemetry.addData("Robot x", getRobotX());
        telemetry.addData("Robot y", getRobotY());*/
    }
}
