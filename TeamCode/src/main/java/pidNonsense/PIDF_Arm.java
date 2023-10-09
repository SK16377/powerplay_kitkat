package pidNonsense;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Disabled
@TeleOp (group = "tests")
public class PIDF_Arm extends OpMode {
    private PIDController controller;

    // p = increase if not reaching target position
    public static double p = 0.007, i = 0, d = 0.0001; // d = dampener (dampens arm movement woah). ignore i
    public static double f = 0.06;  // prevents arm from falling from gravity

    public static int target = 0; // target position
    private final double ticks_in_degree = 700/180.0; //look this up later please :"" 537.6 / 360.0   700/ 180.0

    private DcMotorEx arm_motor;

    @Override
    public void init(){

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        arm_motor = hardwareMap.get(DcMotorEx.class,"w");
    }

    @Override
    public void loop(){
        controller.setPID(p, i, d);
        int armPos = arm_motor.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / ticks_in_degree)) * f;

        double power = pid + ff;

        arm_motor.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
