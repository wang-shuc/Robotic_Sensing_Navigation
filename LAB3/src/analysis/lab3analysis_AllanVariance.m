%% This section is to open and read rosbag
clc
clear all
close all

% Change the file path to the correct path if you are running this script
% Otherwise the script will not read the correct file/FileNotFound 
bagfile = rosbag('C:\Users\ALISA\Desktop\2022-10-11-20-15-04.bag');
imufile = select(bagfile, 'Topic', '/imu');
imudata = readMessages(imufile, 'DataFormat', 'struct');

%% Convert struct into matrix format
magnetic_field = cell2mat(cellfun(@(m) [m.MagField.MagneticField_.X m.MagField.MagneticField_.Y m.MagField.MagneticField_.Z],imudata,'UniformOutput',false));
quaternion = cell2mat(cellfun(@(m) [m.IMU.Orientation.X m.IMU.Orientation.Y m.IMU.Orientation.Z m.IMU.Orientation.W],imudata,'UniformOutput',false));
angular_velocity = cell2mat(cellfun(@(m) [m.IMU.AngularVelocity.X m.IMU.AngularVelocity.Y m.IMU.AngularVelocity.Z],imudata,'UniformOutput',false));
linear_acceleration = cell2mat(cellfun(@(m) [m.IMU.LinearAcceleration.X m.IMU.LinearAcceleration.Y m.IMU.LinearAcceleration.Z],imudata,'UniformOutput',false));


%% Processing Allan Variance
% Draw the graph for Angular Velocity X,Y,Z
GraphTitle = ["(X)", "(Y)", "(Z)"];

for col = 1:3
    Fs = 40; % For lab3 40Hz data output rate.
    t0 = 1/Fs; 
    omega = angular_velocity(:,col);
    
    theta = cumsum(omega, 1)*t0;
    maxNumM = 100;
    L = size(theta, 1);
    maxM = 2.^floor(log2(L/2));
    m = logspace(log10(1), log10(maxM), maxNumM).';
    m = ceil(m); % m must be an integer.
    m = unique(m); % Remove duplicates.
    
    tau = m*t0;
    
    avar = zeros(numel(m), 1);
    for i = 1:numel(m)
        mi = m(i);
        avar(i,:) = sum( ...
            (theta(1+2*mi:L) - 2*theta(1+mi:L-mi) + theta(1:L-2*mi)).^2, 1);
    end
    avar = avar ./ (2*tau.^2 .* (L - 2*m));
    adev = sqrt(avar);
    
    % Find the index where the slope of the log-scaled Allan deviation is equal
    % to the slope specified.
    slope = -0.5;
    logtau = log10(tau);
    logadev = log10(adev);
    dlogadev = diff(logadev) ./ diff(logtau);
    [~, i] = min(abs(dlogadev - slope));
    
    % Find the y-intercept of the line.
    b = logadev(i) - slope*logtau(i);
    
    % Determine the angle random walk coefficient from the line.
    logN = slope*log(1) + b;
    N = 10^logN
    
    % Plot the results.
    tauN = 1;
    lineN = N ./ sqrt(tau);
    % Find the index where the slope of the log-scaled Allan deviation is equal
    % to the slope specified.
    slope = 0.5;
    logtau = log10(tau);
    logadev = log10(adev);
    dlogadev = diff(logadev) ./ diff(logtau);
    [~, i] = min(abs(dlogadev - slope));
    
    % Find the y-intercept of the line.
    b = logadev(i) - slope*logtau(i);
    
    % Determine the rate random walk coefficient from the line.
    logK = slope*log10(3) + b;
    K = 10^logK
    
    % Plot the results.
    tauK = 3;
    lineK = K .* sqrt(tau/3);

    % Find the index where the slope of the log-scaled Allan deviation is equal
    % to the slope specified.
    slope = 0;
    logtau = log10(tau);
    logadev = log10(adev);
    dlogadev = diff(logadev) ./ diff(logtau);
    [~, i] = min(abs(dlogadev - slope));
    
    % Find the y-intercept of the line.
    b = logadev(i) - slope*logtau(i);
    
    % Determine the bias instability coefficient from the line.
    scfB = sqrt(2*log(2)/pi);
    logB = b - log10(scfB);
    B = 10^logB
    
    % Plot the results.
    tauB = tau(i);
    lineB = B * scfB * ones(size(tau));
    
    tauParams = [tauN, tauK, tauB];
    params = [N, K, scfB*B];
    figure(col)
    loglog(tau, adev, tau, [lineN, lineK, lineB], '--', ...
        tauParams, params, 'o')
    title('Allan Deviation with Noise Parameters - Angular Velocity '+GraphTitle(col))
    xlabel('Averaging Time, \tau (s)')
    ylabel('Allan Deviation, \sigma(\tau) (rad/s)')
    legend('$\sigma (rad/s)$', '$\sigma_N ((rad/s)/\sqrt{Hz})$', ...
        '$\sigma_K ((rad/s)\sqrt{Hz})$', '$\sigma_B (rad/s)$', 'Interpreter', 'latex')
    text(tauParams, params, {'N', 'K', '0.664B'})
    grid on
    % axis equal
end

%Draw the graph for linear acceleration X,Y,Z
for col = 1:3
    Fs = 40; % For lab3 40Hz data output rate.
    t0 = 1/Fs; 
    omega = linear_acceleration(:,col);
    
    theta = cumsum(omega, 1)*t0;
    maxNumM = 100;
    L = size(theta, 1);
    maxM = 2.^floor(log2(L/2));
    m = logspace(log10(1), log10(maxM), maxNumM).';
    m = ceil(m); % m must be an integer.
    m = unique(m); % Remove duplicates.
    
    tau = m*t0;
    
    avar = zeros(numel(m), 1);
    for i = 1:numel(m)
        mi = m(i);
        avar(i,:) = sum( ...
            (theta(1+2*mi:L) - 2*theta(1+mi:L-mi) + theta(1:L-2*mi)).^2, 1);
    end
    avar = avar ./ (2*tau.^2 .* (L - 2*m));
    adev = sqrt(avar);

    % Find the index where the slope of the log-scaled Allan deviation is equal
    % to the slope specified.
    slope = -0.5;
    logtau = log10(tau);
    logadev = log10(adev);
    dlogadev = diff(logadev) ./ diff(logtau);
    [~, i] = min(abs(dlogadev - slope));
    
    % Find the y-intercept of the line.
    b = logadev(i) - slope*logtau(i);
    
    % Determine the angle random walk coefficient from the line.
    logN = slope*log(1) + b;
    N = 10^logN
    
    % Plot the results.
    tauN = 1;
    lineN = N ./ sqrt(tau);
    % Find the index where the slope of the log-scaled Allan deviation is equal
    % to the slope specified.
    slope = 0.5;
    logtau = log10(tau);
    logadev = log10(adev);
    dlogadev = diff(logadev) ./ diff(logtau);
    [~, i] = min(abs(dlogadev - slope));
    
    % Find the y-intercept of the line.
    b = logadev(i) - slope*logtau(i);
    
    % Determine the rate random walk coefficient from the line.
    logK = slope*log10(3) + b;
    K = 10^logK
    
    % Plot the results.
    tauK = 3;
    lineK = K .* sqrt(tau/3);

    % Find the index where the slope of the log-scaled Allan deviation is equal
    % to the slope specified.
    slope = 0;
    logtau = log10(tau);
    logadev = log10(adev);
    dlogadev = diff(logadev) ./ diff(logtau);
    [~, i] = min(abs(dlogadev - slope));
    
    % Find the y-intercept of the line.
    b = logadev(i) - slope*logtau(i);
    
    % Determine the bias instability coefficient from the line.
    scfB = sqrt(2*log(2)/pi);
    logB = b - log10(scfB);
    B = 10^logB
    
    % Plot the results.
    tauB = tau(i);
    lineB = B * scfB * ones(size(tau));
    
    tauParams = [tauN, tauK, tauB];
    params = [N, K, scfB*B];
    figure(3+col)
    loglog(tau, adev, tau, [lineN, lineK, lineB], '--', ...
        tauParams, params, 'o')
    title('Allan Deviation with Noise Parameters - Linear Acceleration '+GraphTitle(col))
    xlabel('Averaging Time, \tau (s)')
    ylabel('Allan Deviation, \sigma(\tau) (rad/s)')
    legend('$\sigma (m/s^2)$', '$\sigma_N ((m/s^2)/\sqrt{Hz})$', ...
        '$\sigma_K ((m/s^2)\sqrt{Hz})$', '$\sigma_B (m/s^2)$', 'Interpreter', 'latex')
    text(tauParams, params, {'N', 'K', '0.664B'})
    grid on
    % axis equal
end



