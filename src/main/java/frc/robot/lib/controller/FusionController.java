// package frc.robot.lib.controller;

// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.GenericHID.RumbleType;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj2.command.button.POVButton;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import java.util.HashMap;
// import java.util.Map;
// import java.util.stream.Collectors;

// public class FusionController {
//   private int port;

//   private final Joystick joystick;

//   private final Trigger A;
//   private final Trigger B;
//   private final Trigger X;
//   private final Trigger Y;
//   private final Trigger leftBumper;
//   private final Trigger rightBumper;
//   private final Trigger leftTrigger;
//   private final Trigger rightTrigger;
//   private final Trigger back;
//   private final Trigger start;
//   private final Trigger leftJoystick;
//   private final Trigger rightJoystick;
//   private final Trigger dPadUp;
//   private final Trigger dPadRight;
//   private final Trigger dPadDown;
//   private final Trigger dPadLeft;
//   private final Trigger dPadUpRight;
//   private final Trigger dPadUpLeft;
//   private final Trigger dPadDownRight;
//   private final Trigger dPadDownLeft;

//   private final Axis leftXAxis;
//   private final Axis leftYAxis;
//   private final Axis rightXAxis;
//   private final Axis rightYAxis;
//   private final Axis dPadXAxis;
//   private final Axis dPadYAxis;

//   /**
//    * @param port The port the controller is on
//    */
//   public FusionController(int port) {
//     this.port = port;

//     joystick = new Joystick(port);

//     A = new JoystickButton(joystick, 1);
//     B = new JoystickButton(joystick, 2);
//     X = new JoystickButton(joystick, 3);
//     Y = new JoystickButton(joystick, 4);
//     leftBumper = new JoystickButton(joystick, 5);
//     rightBumper = new JoystickButton(joystick, 6);
//     leftTrigger = new JoystickAxis(joystick, 7).;
//     rightTrigger = new JoystickAxis(joystick, 8);
//     back = new JoystickButton(joystick, 9);
//     start = new JoystickButton(joystick, 10);
//     leftJoystick = new JoystickButton(joystick, 11);
//     rightJoystick = new JoystickButton(joystick, 12);
//     dPadUp = new POVButton(joystick, 0);
//     dPadRight = new POVButton(joystick, 90);
//     dPadDown = new POVButton(joystick, 180);
//     dPadLeft = new POVButton(joystick, 270);
//     dPadUpRight = new POVButton(joystick, 45);
//     dPadUpLeft = new POVButton(joystick, 315);
//     dPadDownRight = new POVButton(joystick, 135);
//     dPadDownLeft = new POVButton(joystick, 225);

//     leftXAxis = new JoystickAxis(joystick, 0);
//     leftYAxis = new JoystickAxis(joystick, 1);
//     rightXAxis = new JoystickAxis(joystick, 2);
//     rightYAxis = new JoystickAxis(joystick, 3);
//     dPadXAxis = new JoystickAxis(joystick, 4);
//     dPadYAxis = new JoystickAxis(joystick, 5);
//     leftYAxis.setInverted(true);
//     rightYAxis.setInverted(true);
//     dPadYAxis.setInverted(true);

//   }

//   public void rumbleController(double value) {
//     joystick.setRumble(RumbleType.kBothRumble, value);
//   }

//   public Trigger getA() {
//     return A;
//   }

//   public Trigger getB() {
//     return B;
//   }

//   public Trigger getX() {
//     return X;
//   }

//   public Trigger getY() {
//     return Y;
//   }

//   public Trigger getLeftBumper() {
//     return leftBumper;
//   }

//   public Trigger getRightBumper() {
//     return rightBumper;
//   }

//   public Trigger getLeftTrigger() {
//     return leftTrigger;
//   }

//   public Trigger getRightTrigger() {
//     return rightTrigger;
//   }

//   public Trigger getBack() {
//     return back;
//   }

//   public Trigger getStart() {
//     return start;
//   }

//   public Trigger getLeftJoystick() {
//     return leftJoystick;
//   }

//   public Trigger getRightJoystick() {
//     return rightJoystick;
//   }

//   public Trigger getDPadUp() {
//     return dPadUp;
//   }

//   public Trigger getDPadRight() {
//     return dPadRight;
//   }

//   public Trigger getDPadDown() {
//     return dPadDown;
//   }

//   public Trigger getDPadLeft() {
//     return dPadLeft;
//   }

//   public Trigger getDPadUpRight() {
//     return dPadUpRight;
//   }

//   public Trigger getDPadUpLeft() {
//     return dPadUpLeft;
//   }

//   public Trigger getDPadDownRight() {
//     return dPadDownRight;
//   }

//   public Trigger getDPadDownLeft() {
//     return dPadDownLeft;
//   }

//   public Axis getLeftXAxis() {
//     return leftXAxis;
//   }

//   public Axis getLeftYAxis() {
//     return leftYAxis;
//   }

//   public Axis getRightXAxis() {
//     return rightXAxis;
//   }

//   public Axis getRightYAxis() {
//     return rightYAxis;
//   }

//   public Axis getDPadXAxis() {
//     return dPadXAxis;
//   }

//   public Axis getDPadYAxis() {
//     return dPadYAxis;
//   }
// }
