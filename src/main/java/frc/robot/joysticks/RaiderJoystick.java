package frc.robot.joysticks;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public abstract class RaiderJoystick extends Joystick {

    /**
     * Construct an instance of a joystick.
     *
     * @param port The port index on the Driver Station that the joystick is plugged
     *             into.
     */
    public RaiderJoystick(int port) {
        super(port);
    }

    public static class Trigger extends edu.wpi.first.wpilibj2.command.button.Button {
        public static final double DEAD_ZONE = 0.05;

        private final int axisPort;
        private final GenericHID parent;

        public Trigger(GenericHID parent, int axisPort) {
            this.parent = parent;
            this.axisPort = axisPort;
        }


        @Override
        public boolean get() {
            return getAxis() >= 0.6;
        }

        /**
         * Get the axis of the trigger
         *
         * @return Between 0-1
         */
        public double getAxis() {
            return deadZone(parent.getRawAxis(axisPort), DEAD_ZONE);
        }
    }

    public static class ThumbStick extends JoystickButton {
        public static final double DEAD_ZONE = 0.05;
        private final int xAxisPort;
        private final int yAxisPort;

        private final GenericHID parent;

        public ThumbStick(GenericHID parent, int buttonNumber, int xAxisPort, int yAxisPort) {
            super(parent, buttonNumber);
            this.parent = parent;
            this.xAxisPort = xAxisPort;
            this.yAxisPort = yAxisPort;
        }

        public double getXAxis() {
            return deadZone(parent.getRawAxis(xAxisPort), DEAD_ZONE);
        }

        public double getYAxis() {
            return -deadZone(parent.getRawAxis(yAxisPort), DEAD_ZONE);
        }
    }

    public static class DPad {
        public final GenericHID parent;
        public final DPadButton up = new DPadButton(this, DPadDirection.UP);
        public final DPadButton down = new DPadButton(this, DPadDirection.DOWN);
        public final DPadButton left = new DPadButton(this, DPadDirection.LEFT);
        public final DPadButton right = new DPadButton(this, DPadDirection.RIGHT);

        public DPad(GenericHID parent) {
            this.parent = parent;
        }

        public int angle() {
            return parent.getPOV();
        }

        public enum DPadDirection {
            UP(0),
            RIGHT(90),
            DOWN(180),
            LEFT(270);

            public final int angle;

            DPadDirection(int angle) {
                this.angle = angle;
            }
        }

        public static class DPadButton extends edu.wpi.first.wpilibj2.command.button.Button {
            public final DPad parent;
            private final DPadDirection direction;

            public DPadButton(DPad parent, DPadDirection direction) {
                this.parent = parent;
                this.direction = direction;
            }

            @Override
            public boolean get() {
                return parent.angle() == direction.angle;
            }
        }
    }

    public static double deadZone(double value, double deadZone) {
        if (Math.abs(deadZone) < value) {
            return 0.0;
        }
        return value;
    }
}
