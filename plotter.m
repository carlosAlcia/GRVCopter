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
plot(Time, FORCE_X_DES_ID)
hold on
plot(Time, FORCE_Y_DES_ID)
plot(Time, FORCE_Z_DES_ID)
title("Forces")
grid minor 
legend("Fx", "Fy", "Fz");


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

%%

figure
plot(Time, -POS_Z_ID)
hold on 
plot(Time, -DES_POS_Z_ID)
title("PosZ")
grid minor 
legend("Real", "Target");
figure
plot(Time, FORCE_Z_DES_ID)
hold on
plot(Time, DES_POS_Z_ID- POS_Z_ID)
title("Force Vs Z Error")
grid minor 
legend("Force", "Z Error")


% 
% 
figure
plot(Time, PWM1_ID)
hold on 
plot(Time, PWM2_ID)
plot(Time, PWM3_ID)
plot(Time, PWM4_ID)
plot(Time, PWM5_ID)
plot(Time, PWM6_ID)

title("PWM")
grid minor 
legend("M1", "M2", "M3", "M4", "M5", "M6");
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
%%
% figure
% plot(Time, ROLL_ID)
% hold on 
% plot(Time, DES_ROLL_ID*3.14/180)
% title("Roll")
% grid minor 
% legend("Real", "Target");
% 
% figure
% plot(Time, PITCH_ID)
% hold on 
% plot(Time, DES_PITCH_ID*3.14/180)
% title("Pitch")
% grid minor 
% legend("Real", "Target");
% 
% figure
% plot(Time, YAW_ID)
% hold on 
% plot(Time, DES_YAW_ID)
% title("Yaw")
% grid minor 
% legend("Real", "Target");

