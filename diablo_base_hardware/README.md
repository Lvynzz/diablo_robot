# `diablo_base_hardware`

`DiabloSystemHardware` adapts the two wheel interfaces expected by
`diff_drive_controller` to the official Diablo vehicle API:

- command: `/diablo/MotionCmd` (`MotionCtrl`)
- feedback: `/diablo/sensor/Motors` (`LegMotors`)

The adapter converts wheel angular velocities to `value.forward` and
`value.left` using:

```text
linear  = wheel_radius * (left + right) / 2
angular = wheel_radius * (right - left) / track_width
```

Feedback positions are unwrapped with `enc_rev * 2*pi`.  The default feedback
signs are `left=+1` and `right=-1`, matching the SDK's wheel direction
convention.  The full-body launch exposes these as
`left_feedback_sign`/`right_feedback_sign` for a physical direction check:
after a local reset, a real forward motion must increase `x`. All geometry,
signs, limits, topic names, and the feedback timeout are hardware parameters
in the URDF.

On activation the adapter sends one `mode_mark=true`, `stand_mode=false`
message so the official driver requests crawling mode.  During operation it
sends vehicle-level motion commands at 25 Hz by default, matching the official
teleop rate and avoiding unnecessary serial traffic; the ros2_control loop can
run faster for feedback and odometry.  Deactivation sends a zero command.
