%% This section is to close exist workspace and read from ros bagfile
clc
clear all
close all

bagfile = rosbag('C:\Users\ALISA\Desktop\bags\ruggles.bag');
imufile = select(bagfile, 'Topic', '/imu');
imudata = readMessages(imufile, 'DataFormat', 'struct');
gpsfile = select(bagfile, 'Topic', '/gps');
gpsdata = readMessages(gpsfile, 'DataFormat', 'struct');

magnetic_field = cell2mat(cellfun(@(m) [m.MagField.MagneticField_.X m.MagField.MagneticField_.Y m.MagField.MagneticField_.Z],imudata,'UniformOutput',false));
quaternion = cell2mat(cellfun(@(m) [m.IMU.Orientation.X m.IMU.Orientation.Y m.IMU.Orientation.Z m.IMU.Orientation.W],imudata,'UniformOutput',false));
angular_velocity = cell2mat(cellfun(@(m) [m.IMU.AngularVelocity.X m.IMU.AngularVelocity.Y m.IMU.AngularVelocity.Z],imudata,'UniformOutput',false));
linear_acceleration = cell2mat(cellfun(@(m) [m.IMU.LinearAcceleration.X m.IMU.LinearAcceleration.Y m.IMU.LinearAcceleration.Z],imudata,'UniformOutput',false));
time = cellfun(@(m) str2double(strcat(num2str(m.Header.Stamp.Sec-imudata{1,1}.Header.Stamp.Sec),'.',num2str(m.Header.Stamp.Nsec))),imudata);

%% Magnetic Field Soft Iron & Hard Iron Calibration
mag_x = magnetic_field(:,1);
mag_y = magnetic_field(:,2);
mag_z = magnetic_field(:,3);


figure(1)
scatter(mag_x,mag_y,'.')
grid on
axis equal
xline(0)
yline(0)
xlim([-0.2, 1.2])
xlabel("Magnetic field x (Gauss)")
ylabel("Magnetic field y (Gauss)")
title("Raw Magnetic Field X vs.Y")

%Start calibration 
ellipse = fit_ellipse(mag_x,mag_y);
theta = -ellipse.phi
hard_iron_x = ellipse.X0_in
hard_iron_y = ellipse.Y0_in

Si = [cos(theta) sin(theta) 0;
     -sin(theta) cos(theta) 0;
     0 0 1];

b = transpose([mag_x - hard_iron_x mag_y - hard_iron_y mag_z - 0]);
calibration = Si * b;
calibrated_x = calibration(1,:);
calibrated_y = calibration(2,:);

figure(2)
hold on
scatter(mag_x,mag_y,'.')
scatter(calibrated_x, calibrated_y, '.')
grid on
axis equal
xline(0)
yline(0)
xlim([-0.4, 1.2])
xlabel("Magnetic field x (Gauss)")
ylabel("Magnetic field y (Gauss)")
legend("Calibrate before", "Calibrate after")
title("Magnetic Field Calibration Before and After")
hold off

