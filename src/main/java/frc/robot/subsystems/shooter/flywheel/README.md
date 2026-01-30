# Flywheel tuning (Talon FX velocity control)

Follow this order when tuning (CTRE / Phoenix 6):

1. Set all gains to zero (kP, kI, kD, kV, kS).
2. Increase **kS** until the motor just barely starts to move (static friction).
3. Increase **kV** until measured velocity tracks the setpoint well (velocity feedforward).
4. Increase **kP** until the response starts to oscillate, then back off slightly.
5. Increase **kD** to reduce overshoot/jitter without adding noise.
