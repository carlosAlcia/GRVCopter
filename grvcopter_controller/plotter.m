plot(Time, TorqueRoll)
hold on
plot(Time, TorquePitch)
plot(Time, TorqueYaw)
title("Torques")
grid minor 
legend("T_rll", "T_pit", "T_yaw");

figure
plot(Time, Roll)
hold on 
plot(Time, Pitch)
plot(Time, Yaw)
title("Angles")
grid minor 
legend("Roll", "Pitch", "Yaw");

figure
plot(Time, PWM0)
hold on 
plot(Time, PWM1)
plot(Time, PWM2)
plot(Time, PWM3)
plot(Time, PWM4)
plot(Time, PWM5)

title("PWM")
grid minor 
legend("M1", "M2", "M3", "M4", "M5", "M6");

figure
plot(Time, TorqueRoll)
hold on
plot(Time, Roll)
title("Torque Vs Angle")
grid minor 
legend("Torque R", "Roll")

figure
plot(Time, TorquePitch)
hold on
plot(Time, Pitch)
title("Torque Vs Angle")
grid minor 
legend("Torque P", "Pitch")

figure
plot(Time, TorqueYaw)
hold on
plot(Time, Yaw)
title("Torque Vs Angle")
grid minor 
legend("Torque Y", "Yaw")

