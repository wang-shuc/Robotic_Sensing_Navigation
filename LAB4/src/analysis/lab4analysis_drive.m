%% Read from Ros bag
clc
clear all
close all

bagfile = rosbag('C:\Users\ALISA\Desktop\bags\backbay_tour.bag');
imufile = select(bagfile, 'Topic', '/imu');
imudata = readMessages(imufile, 'DataFormat', 'struct');
gpsfile = select(bagfile, 'Topic', '/gps');
gpsdata = readMessages(gpsfile, 'DataFormat', 'struct');

magnetic_field = cell2mat(cellfun(@(m) [m.MagField.MagneticField_.X m.MagField.MagneticField_.Y m.MagField.MagneticField_.Z],imudata,'UniformOutput',false));
quaternion = cell2mat(cellfun(@(m) [m.IMU.Orientation.X m.IMU.Orientation.Y m.IMU.Orientation.Z m.IMU.Orientation.W],imudata,'UniformOutput',false));
angular_velocity = cell2mat(cellfun(@(m) [m.IMU.AngularVelocity.X m.IMU.AngularVelocity.Y m.IMU.AngularVelocity.Z],imudata,'UniformOutput',false));
linear_acceleration = cell2mat(cellfun(@(m) [m.IMU.LinearAcceleration.X m.IMU.LinearAcceleration.Y m.IMU.LinearAcceleration.Z],imudata,'UniformOutput',false));
time = cellfun(@(m) str2double(strcat(num2str(m.Header.Stamp.Sec-imudata{1,1}.Header.Stamp.Sec),'.',num2str(m.Header.Stamp.Nsec))),imudata);

utm = cell2mat(cellfun(@(m) [m.UTMEasting m.UTMNorthing],gpsdata,'UniformOutput',false));
gps_time = cellfun(@(m) str2double(strcat(num2str(m.Header.Stamp.Sec-gpsdata{1,1}.Header.Stamp.Sec),'.',num2str(m.Header.Stamp.Nsec))),gpsdata);

%% Yaw
mag_x = magnetic_field(:,1);
mag_y = magnetic_field(:,2);
mag_z = magnetic_field(:,3);

eular = quat2eul(quaternion);
eular_yaw = eular(:,1);

% Raw yaw from uncalibrated magnetic field
mag_yaw = transpose(atan2(mag_x,mag_y));

% Calibrating magnetometer similar to previous step
ellipse = fit_ellipse(mag_x,mag_y)
theta = -ellipse.phi;
hard_iron_x = ellipse.X0_in;
hard_iron_y = ellipse.Y0_in;

Si = [cos(theta) sin(theta) 0;
     -sin(theta) cos(theta) 0;
     0 0 1];

b = transpose([mag_x - hard_iron_x mag_y - hard_iron_y mag_z - 0]);
calibration = Si * b;
calibrated_x = calibration(1,:);
calibrated_y = calibration(2,:);

% Calculated calibrated yaw angle from calibrated magnetic field x and y
calibrated_yaw = transpose(atan2(calibrated_x,calibrated_y));

angular_yaw = angular_velocity(:,3);
% Cumulative trapezoidal numerical integration --- cumtrapz()
angle_yaw = cumtrapz(time,angular_yaw);
% Selected Alpha value for Complementary Filter II
a = 0.03;
% Lecture Powerpoint Complementary Filter II
filter = a * unwrap(calibrated_yaw) + (1-a) * (angle_yaw + calibrated_yaw(1));
% Wrap angle from radius to Pi
filtered = wrapToPi(filter);


figure(1)
hold on
grid on
plot(time, mag_yaw*180/pi,'.')
plot(time, calibrated_yaw*180/pi,'.')
title("Raw Magnetic Yaw and Calibrated Magnetic Field Yaw")
legend("Raw","Calibrated")
xlabel("Time (s)")
ylabel("Yaw (degree)")
hold off



figure(2)
hold on
grid on
plot(time,wrapToPi(angle_yaw)*180/pi,'.')
plot(time,calibrated_yaw*180/pi,'.')
plot(time,eular_yaw*180/pi,'.')
plot(time,filtered*180/pi,'.')
title("Yaw Angle")
xlabel("time (s)")
ylabel("Yaw (degrees)")
legend("Gyro High Pass Filter","Magnetometer","IMU Low Pass Filter","Complementary Filter")
hold off

figure(3)
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

%% Forward acceleration
linear_acceleration_x = linear_acceleration(:,1);
figure(4)
plot(time, linear_acceleration_x, '.');
grid on
xlabel("Time (s)")
ylabel("Linear Acceleration in x direction (m/s^2)")
title("Forward Acceleration")

%% Forward velocity
imu_velocity = cumtrapz(time, linear_acceleration_x);

gps_x = gradient(utm(:,1)) ./ gradient(gps_time);
gps_y = gradient(utm(:,2)) ./ gradient(gps_time);
gps_velocity = sqrt(gps_x .^2 + gps_y .^2);

figure(5)
hold on
grid on
plot(time, imu_velocity, '.')
plot(gps_time, gps_velocity, '.')
title("Forward Velocity from IMU and  GPS")
xlabel("Time (s)")
ylabel("Velocity (m/s)")
legend("IMU data", "GPS data")


%% Calibrate velocity
mean_imu_acc = mean(linear_acceleration_x);
imu_calibrated = linear_acceleration_x - mean_imu_acc;

i = smoothdata(linear_acceleration_x, 'gaussian', 200);
j = gradient(i);
for counter = 1 : length(j)
    if 0.0004 > abs(j(counter))
        result(counter) = 0;
    else
        result(counter) = imu_calibrated(counter);
    end
end

imu_result = transpose(cumtrapz(time, result));

figure(5)
subplot(2,1,1)
grid on
hold on
%plot(time, imu_result, '.')
plot(gps_time, gps_velocity,'blue')
title("Forward Velocity with Calibrated IMU")
ylabel("Velocity (m/s)")
legend("GPS")
hold off

subplot(2,1,2)
grid on
hold on
plot(time, imu_result, 'black')
%plot(gps_time, gps_velocity, '.')
%title("Forward Velocity with Calibrated IMU")
xlabel("Time (s)")
ylabel("Velocity (m/s)")
legend("IMU")
hold off

%% Dead Reckoning with IMU
imu_position = cumtrapz(time,imu_result);
gps_position = cumtrapz(gps_time, gps_velocity);

gps_position_x = utm(:,1) - utm(1,1);
gps_position_y = utm(:,2) - utm(1,2);

figure(6)
grid on
hold on
plot(time, imu_position)
plot(gps_time, gps_position)
title("IMU and GPS Displacement")
xlabel("Time (s)")
ylabel("Displacement (m)")
legend("IMU", "GPS")
hold off

omega = angular_velocity(:,3);
x_diff = gps_velocity;
omegax_diff = omega .* imu_velocity;
y_2diff = linear_acceleration(:,2);

figure(7)
grid on
hold on
plot(time, y_2diff, '.')
plot(time, omegax_diff, '.')
title("Comparison of Acceleration")
xlabel("Time (s)")
ylabel("Acceleration (m/s^2)")
legend("y''", '\omegaX')
hold off

imu_east_velocity = -imu_velocity .* cos(filtered);
imu_north_velocity = imu_velocity .* sin(filtered);
imu_easting = cumtrapz(time, imu_east_velocity);
imu_northing = cumtrapz(time, imu_north_velocity);

figure(8)
grid on
hold on
subplot(2,1,1)
plot(gps_position_x, gps_position_y, '.')
legend("GPS")
title("Driving Route")

ylabel("Northing (m)")
subplot(2,1,2)
plot(imu_easting,imu_northing,'.')
xlabel("Easting (m)")
ylabel("Northing (m)")
legend("IMU")
hold off

%% IMU Offset
J = omegax_diff;
K = y_2diff(1:end) - omegax_diff(1:end);
offset = linsolve(J, K)