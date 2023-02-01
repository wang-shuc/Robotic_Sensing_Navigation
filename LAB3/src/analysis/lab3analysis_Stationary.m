%% This section is to open and read rosbag
clc
clear all
close all

% Change the file path to the correct path if you are running this script
% Otherwise the script will not read the correct file/FileNotFound 
bagfile = rosbag('C:\Users\ALISA\Desktop\stationary.bag');
imufile = select(bagfile, 'Topic', '/imu');
imudata = readMessages(imufile, 'DataFormat', 'struct');

%% Convert struct into matrix format
magnetic_field = cell2mat(cellfun(@(m) [m.MagField.MagneticField_.X m.MagField.MagneticField_.Y m.MagField.MagneticField_.Z],imudata,'UniformOutput',false));
quaternion = cell2mat(cellfun(@(m) [m.IMU.Orientation.X m.IMU.Orientation.Y m.IMU.Orientation.Z m.IMU.Orientation.W],imudata,'UniformOutput',false));
angular_velocity = cell2mat(cellfun(@(m) [m.IMU.AngularVelocity.X m.IMU.AngularVelocity.Y m.IMU.AngularVelocity.Z],imudata,'UniformOutput',false));
linear_acceleration = cell2mat(cellfun(@(m) [m.IMU.LinearAcceleration.X m.IMU.LinearAcceleration.Y m.IMU.LinearAcceleration.Z],imudata,'UniformOutput',false));
time = cellfun(@(m) str2double(strcat(num2str(m.Header.Stamp.Sec-imudata{1,1}.Header.Stamp.Sec),'.',num2str(m.Header.Stamp.Nsec))),imudata);


%% Analysis stationary data
% The below 4 figures are quaternion with X Y Z W.
figure(1)
Quaternion_X = quaternion(:,1);
Quaternion_Mean_X = mean(Quaternion_X);
Quaternion_std_X = std(Quaternion_X);
plot(time,quaternion(:,1))
title("IMU Quaternion.Orientation X")
yline(Quaternion_Mean_X,'r')
yline(Quaternion_std_X,'g')
xlabel("Time(sec)")
ylabel("Quaternion")
txt1 = ['Mean: ' num2str(Quaternion_Mean_X)];
txt2 = ['Standard Deviation: ' num2str(Quaternion_std_X)];
legend({"Quaternion Orientation X",txt1,txt2},'Location','best')


figure(2)
Quaternion_Y = quaternion(:,2);
Quaternion_Mean_Y = mean(Quaternion_Y);
Quaternion_std_Y = std(Quaternion_Y);
plot(time,quaternion(:,2))
title("IMU Quaternion.Orientation Y")
yline(Quaternion_Mean_Y,'r')
yline(Quaternion_std_Y,'g')
xlabel("Time(sec)")
ylabel("Quaternion")
txt1 = ['Mean: ' num2str(Quaternion_Mean_Y)];
txt2 = ['Standard Deviation: ' num2str(Quaternion_std_Y)];
legend({"Quaternion Orientation Y",txt1,txt2},'Location','best')


figure(3)
Quaternion_Z = quaternion(:,3);
Quaternion_Mean_Z = mean(Quaternion_Z);
Quaternion_std_Z = std(Quaternion_Z);
plot(time,quaternion(:,3))
title("IMU Quaternion.Orientation Z")
yline(Quaternion_Mean_Z,'r')
yline(Quaternion_std_Z,'g')
xlabel("Time(sec)")
ylabel("Quaternion")
txt1 = ['Mean: ' num2str(Quaternion_Mean_Z)];
txt2 = ['Standard Deviation: ' num2str(Quaternion_std_Z)];
legend({"Quaternion Orientation Z",txt1,txt2},'Location','best')

figure(4)
Quaternion_W = quaternion(:,4);
Quaternion_Mean_W = mean(Quaternion_W);
Quaternion_std_W = std(Quaternion_W);
plot(time,quaternion(:,4))
title("IMU Quaternion.Orientation \omega")
yline(Quaternion_Mean_W,'r')
yline(Quaternion_std_W,'g')
xlabel("Time(sec)")
ylabel("Quaternion")
txt1 = ['Mean: ' num2str(Quaternion_Mean_W)];
txt2 = ['Standard Deviation: ' num2str(Quaternion_std_W)];
legend({"Quaternion Orientation \omega",txt1,txt2},'Location','best')

%% Euler Coordinates 3D view
% Here I convert quaternion back to Euler coordinates using quat2eul()
euler = [quat2eul(quaternion(1,:))];
for counter = 2 : length(quaternion)
    euler = [euler; quat2eul(quaternion(counter,:))];
end

figure(5)
plot3(euler(:,1),euler(:,2),euler(:,3), '.')
grid on
title("Euler Coordinates")
xlabel("Roll (deg)")
ylabel("Pitch (deg)")
zlabel("Yaw (deg)")

roll = euler(:,1);
pitch = euler(:,2);
yaw = euler(:,3);

figure(19)
roll_mean = mean(roll);
roll_std = std(roll);
plot(time,roll);
title("Euler Angles - Roll(x)")
yline(roll_mean, 'r')
yline(roll_std, 'g')
xlabel("Time(sec)")
ylabel("Roll")
txt1 = ['Mean: ' num2str(roll_mean)];
txt2 = ['Standard Deviation: ' num2str(roll_std)];
legend({"Roll(x)", txt1, txt2}, 'Location', 'best')

figure(20)
pitch_mean = mean(pitch);
pitch_std = std(pitch);
plot(time,pitch);
title("Euler Angles - Pitch(y)")
yline(pitch_mean, 'r')
yline(pitch_std, 'g')
xlabel("Time(sec)")
ylabel("Roll")
txt1 = ['Mean: ' num2str(pitch_mean)];
txt2 = ['Standard Deviation: ' num2str(pitch_std)];
legend({"Pitch(y)", txt1, txt2}, 'Location', 'best')

figure(21)
yaw_mean = mean(yaw);
yaw_std = std(yaw);
plot(time,yaw);
title("Euler Angles - Yaw(z)")
yline(yaw_mean, 'r')
yline(yaw_std, 'g')
xlabel("Time(sec)")
ylabel("Roll")
txt1 = ['Mean: ' num2str(yaw_mean)];
txt2 = ['Standard Deviation: ' num2str(yaw_std)];
legend({"Yaw(z)", txt1, txt2}, 'Location', 'best')

%% Angular Velocity 
figure(6)
Angular_X = angular_velocity(:,1);
Angular_Mean_X = mean(Angular_X);
Angular_std_X = std(Angular_X);
plot(time, Angular_X);
title("IMU Angular Velocity X")
yline(Angular_Mean_X, 'r')
yline(Angular_std_X, 'g')
xlabel("Time(sec)")
ylabel("Angular Velocity(rad/s)")
txt1 = ['Mean: ' num2str(Angular_Mean_X)];
txt2 = ['Standard Deviation: ', num2str(Angular_std_X)];
legend({"Angular velocity", txt1, txt2}, 'Location', 'best')

figure(7)
Angular_Y = angular_velocity(:,2);
Angular_Mean_Y = mean(Angular_Y);
Angular_std_Y = std(Angular_Y);
plot(time, Angular_Y);
title("IMU Angular Velocity Y")
yline(Angular_Mean_Y, 'r')
yline(Angular_std_Y, 'g')
xlabel("Time(sec)")
ylabel("Angular Velocity(rad/s)")
txt1 = ['Mean: ' num2str(Angular_Mean_Y)];
txt2 = ['Standard Deviation: ', num2str(Angular_std_Y)];
legend({"Angular velocity", txt1, txt2}, 'Location', 'best')

figure(8)
Angular_Z = angular_velocity(:,3);
Angular_Mean_Z = mean(Angular_Z);
Angular_std_Z = std(Angular_Z);
plot(time, Angular_Z);
title("IMU Angular Velocity Z")
yline(Angular_Mean_Z, 'r')
yline(Angular_std_Z, 'g')
xlabel("Time(sec)")
ylabel("Angular Velocity(rad/s)")
txt1 = ['Mean: ' num2str(Angular_Mean_Z)];
txt2 = ['Standard Deviation: ', num2str(Angular_std_Z)];
legend({"Angular velocity", txt1, txt2}, 'Location', 'best')

figure(9)
plot3(Angular_X, Angular_Y, Angular_Z, '.')
grid on
title("Angular Velocity 3D View")
xlabel("Angular Velocity X (rad/s)")
ylabel("Angular Velocity Y (rad/s)")
zlabel("Angular Velocity Z (rad/s)")

%% Linear Acceleration
figure(10)
Linear_X = linear_acceleration(:,1);
Linear_Mean_X = mean(Linear_X);
Linear_std_X = std(Linear_X);
plot(time, Linear_X);
title("IMU Linear Acceleration X")
yline(Linear_Mean_X, 'r')
yline(Linear_std_X, 'g')
xlabel("Time(sec)")
ylabel("Linear Acceleration(m/s^2)")
txt1 = ['Mean: ' num2str(Linear_Mean_X)];
txt2 = ['Standard Deviation: ', num2str(Linear_std_X)];
legend({"Linear Acceleration", txt1, txt2}, 'Location', 'best')

figure(11)
Linear_Y = linear_acceleration(:,2);
Linear_Mean_Y = mean(Linear_Y);
Linear_std_Y = std(Linear_Y);
plot(time, Linear_Y);
title("IMU Linear Acceleration Y")
yline(Linear_Mean_Y, 'r')
yline(Linear_std_Y, 'g')
xlabel("Time(sec)")
ylabel("Linear Acceleration(m/s^2)")
txt1 = ['Mean: ' num2str(Linear_Mean_Y)];
txt2 = ['Standard Deviation: ', num2str(Linear_std_Y)];
legend({"Linear Acceleration", txt1, txt2}, 'Location', 'best')

figure(12)
Linear_Z = linear_acceleration(:,3);
Linear_Mean_Z = mean(Linear_Z);
Linear_std_Z = std(Linear_Z);
plot(time, Linear_Z);
title("IMU Linear Acceleration Z")
yline(Linear_Mean_Z, 'r')
yline(Linear_std_Z, 'g')
xlabel("Time(sec)")
ylabel("Linear Acceleration(m/s^2)")
txt1 = ['Mean: ' num2str(Linear_Mean_Z)];
txt2 = ['Standard Deviation: ', num2str(Linear_std_Z)];
legend({"Linear Acceleration", txt1, txt2}, 'Location', 'best')

figure(13)
plot3(Linear_X, Linear_Y, Linear_Z, '.')
grid on
title("Linear Acceleration 3D View")
xlabel("Linear Acceleration X (m/s^2)")
ylabel("Linear Acceleration Y (m/s^2)")
zlabel("Linear Acceleration Z (m/s^2)")

%% Magnetic Field
figure(14)
Mag_X = magnetic_field(:,1);
Mag_Mean_X = mean(Mag_X);
Mag_std_X = std(Mag_X);
plot(time, Mag_X);
title("MagneticField X")
yline(Mag_Mean_X, 'r')
yline(Mag_std_X, 'g')
xlabel("Time(sec)")
ylabel("MagneticField X (Teslas)")
txt1 = ['Mean: ' num2str(Mag_Mean_X)];
txt2 = ['Standard Deviation: ', num2str(Mag_std_X)];
legend({"MagneticField", txt1, txt2}, 'Location', 'best')

figure(16)
Mag_Y = magnetic_field(:,2);
Mag_Mean_Y = mean(Mag_Y);
Mag_std_Y = std(Mag_Y);
plot(time, Mag_Y);
title("MagneticField Y")
yline(Mag_Mean_Y, 'r')
yline(Mag_std_Y, 'g')
xlabel("Time(sec)")
ylabel("MagneticField Y (Teslas)")
txt1 = ['Mean: ' num2str(Mag_Mean_Y)];
txt2 = ['Standard Deviation: ', num2str(Mag_std_Y)];
legend({"MagneticField", txt1, txt2}, 'Location', 'best')

figure(17)
Mag_Z = magnetic_field(:,3);
Mag_Mean_Z = mean(Mag_Z);
Mag_std_Z = std(Mag_Z);
plot(time, Mag_Z);
title("MagneticField Z")
yline(Mag_Mean_Z, 'r')
yline(Mag_std_Z, 'g')
xlabel("Time(sec)")
ylabel("MagneticField Z (Teslas)")
txt1 = ['Mean: ' num2str(Mag_Mean_Z)];
txt2 = ['Standard Deviation: ', num2str(Mag_std_Z)];
legend({"MagneticField", txt1, txt2}, 'Location', 'best')

figure(18)
plot3(Mag_X, Mag_Y, Mag_Z, '.')
grid on
title("Magnetic Field 3D View")
xlabel("Magnetic Field Direction X (Teslas)")
ylabel("Magnetic Field Direction Y (Teslas)")
zlabel("Magnetic Field Direction Z (Teslas)")
