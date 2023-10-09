package pidNonsense;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@Disabled
@TeleOp (group = "tests")
public class PIDF_DoubleMotor extends OpMode {
    private PIDController controller;

    // p = increase if not reaching target position
    public static double p = 0, i = 0, d = 0; // d = dampener (dampens arm movement woah). ignore i
    public static double f = 0;  // prevents arm from falling from gravity
    public static double wf = 0;

    public static int Ltarget = 0; // target position
    public static int Rtarget = 0;
    public static int Wtarget = 0;
    private final double ticks_in_degree = 700/180.0; //look this up later please :"" 537.6 / 360.0   700/ 180.0

    private DcMotorEx larm;
    private DcMotorEx rarm;
    private DcMotorEx wrist;

    @Override
    public void init(){
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        larm = hardwareMap.get(DcMotorEx.class,"la");
        rarm = hardwareMap.get(DcMotorEx.class,"ra");
        wrist = hardwareMap.get(DcMotorEx.class,"w");
    }

    @Override
    public void loop(){

        controller.setPID(p, i, d);

        int larmPos = larm.getCurrentPosition();
        int rarmPos = rarm.getCurrentPosition();
        int wristPos = rarm.getCurrentPosition();

        double Lpid = controller.calculate(larmPos, Ltarget);
        double Rpid = controller.calculate(rarmPos, Rtarget);
        double Wpid = controller.calculate(wristPos, Wtarget);

        double Lff = Math.cos(Math.toRadians(Ltarget / ticks_in_degree)) * f;
        double Rff = Math.cos(Math.toRadians(Rtarget / ticks_in_degree)) * f;
        double Wff = Math.cos(Math.toRadians(Wtarget / ticks_in_degree)) * wf;

        double Lpower = Lpid + Lff;
        double Rpower = Rpid + Rff;
        double Wpower = Wpid + Wff;

        larm.setPower(Lpower);
        rarm.setPower(Rpower);
        wrist.setPower(Wpower);

        telemetry.addData("Lpos", larmPos);
        telemetry.addData("Rpos", rarmPos);
        telemetry.addData("Wpos", wristPos);
        telemetry.addData("Ltarget", Ltarget);
        telemetry.addData("Rtarget", Rtarget);
        telemetry.addData("Wtarget", Wtarget);
        telemetry.update();
    }
}
