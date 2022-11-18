import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Swing {

    public Servo leftServoSwing;
    public Servo rightServoSwing;

    public Swing(HardwareMap hardwareMap) {
        leftServoSwing = hardwareMap.get(Servo.class, "leftServoSwing");
        rightServoSwing = hardwareMap.get(Servo.class, "rightServoSwing");

    }

    public void setPosition(double p1){
        leftServoSwing.setPosition(p1);
        rightServoSwing.setPosition(1-p1);
    }
}