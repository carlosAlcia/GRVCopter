% plot(Time, TorqueRoll)
% hold on
% plot(Time, TorquePitch)
% plot(Time, TorqueYaw)
% title("Torques")
% grid minor 
% legend("T_rll", "T_pit", "T_yaw");
% 
% figure
% plot(Time, ForceX)
% hold on
% plot(Time, ForceY)
% plot(Time, ForceZ)
% title("Forces")
% grid minor 
% legend("Fx", "Fy", "Fz");

figure
plot(Time, ForceX)
hold on
plot(Time, ForceY)
plot(Time, ForceZ)
title("Forces")
grid minor 
legend("Fx", "Fy", "Fz");


figure
plot(Time, Pos_X)
hold on 
plot(Time, Des_Pos_X)
title("PosX")
grid minor 
legend("Real", "Target");

figure
plot(Time, Pos_Y)
hold on 
plot(Time, Des_Pos_Y)
title("PosY")
grid minor 
legend("Real", "Target");

figure
plot(Time, Pos_Z)
hold on 
plot(Time, Des_Pos_Z)
title("PosZ")
grid minor 
legend("Real", "Target");

% 
% 
% figure
% plot(Time, PWM0)
% hold on 
% plot(Time, PWM1)
% plot(Time, PWM2)
% plot(Time, PWM3)
% plot(Time, PWM4)
% plot(Time, PWM5)
% 
% title("PWM")
% grid minor 
% legend("M1", "M2", "M3", "M4", "M5", "M6");
% 
% figure
% plot(Time, TorqueRoll)
% hold on
% plot(Time, Roll)
% title("Torque Vs Angle")
% grid minor 
% legend("Torque R", "Roll")
% 
% figure
% plot(Time, TorquePitch)
% hold on
% plot(Time, Pitch)
% title("Torque Vs Angle")
% grid minor 
% legend("Torque P", "Pitch")
% 
% figure
% plot(Time, TorqueYaw)
% hold on
% plot(Time, Yaw)
% title("Torque Vs Angle")
% grid minor 
% legend("Torque Y", "Yaw")

figure
plot(Time, Roll)
hold on 
plot(Time, Des_Roll)
title("Roll")
grid minor 
legend("Real", "Target");

figure
plot(Time, Pitch)
hold on 
plot(Time, Des_Pitch)
title("Pitch")
grid minor 
legend("Real", "Target");

figure
plot(Time, Yaw)
hold on 
plot(Time, Des_Yaw)
title("Yaw")
grid minor 
legend("Real", "Target");

