package frc.robot.subsystems.Lights;

public enum CameraLEDs {
    FrontRight(0,0),
    FrontLeft(0,0),
    Left(0,0),
    Right(0,0),
    Back(0,0);

    public final int StartLED;
    public final int EndLED;

    CameraLEDs(int StartLED, int EndLED){
        this.StartLED = StartLED;
        this.EndLED = EndLED;
    }
    public static CameraLEDs[] values = CameraLEDs.values();

    public static CameraLEDs forIndex(int index){
        return values[index];
    }
}
