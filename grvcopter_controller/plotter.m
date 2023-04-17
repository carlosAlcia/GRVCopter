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

% figure
% plot(Time, ForceX)
% hold on
% plot(Time, ForceY)
% plot(Time, ForceZ)
% title("Forces")
% grid minor 
% legend("Fx", "Fy", "Fz");


figure
plot(Time, POS_X_ID)
hold on 
plot(Time, DES_POS_X_ID)
title("PosX")
grid minor 
legend("Real", "Target");

figure
plot(Time, POS_Y_ID)
hold on 
plot(Time, DES_POS_Y_ID)
title("PosY")
grid minor 
legend("Real", "Target");

figure
plot(Time, POS_Z_ID)
hold on 
plot(Time, DES_POS_Z_ID)
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
plot(Time, ROLL_ID)
hold on 
plot(Time, DES_ROLL_ID)
title("Roll")
grid minor 
legend("Real", "Target");

figure
plot(Time, PITCH_ID)
hold on 
plot(Time, DES_PITCH_ID)
title("Pitch")
grid minor 
legend("Real", "Target");

figure
plot(Time, YAW_ID)
hold on 
plot(Time, DES_YAW_ID)
title("Yaw")
grid minor 
legend("Real", "Target");

