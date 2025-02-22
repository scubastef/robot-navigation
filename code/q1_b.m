% This script runs Q1(b)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Since we are doing just prediction, all the other sensors are disabled.
% This is the default setting.

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q1_b');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);
%drivebotSLAMSystem.setRecommendOptimizationPeriod(10);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. These just show you an example of how to plot the
% results. For your report, you need to look at improving these figures
% including labelling axes, etc.

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.vehicleStateTime, results{1}.optimizationTimes, '*')
hold on

% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory', LineWidth=1)
lgd = legend('x', 'y', '\psi', Location='southwest');
lgd.FontSize = 15;
xlabel('Time step (seconds)')
ylabel('Vehicle State Error')
title('Errors With Odometry Only')

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleStateTime, results{1}.vehicleCovarianceHistory', LineWidth=1)
lgd = legend('x', 'y', '\psi', Location='northwest');
lgd.FontSize = 15;
xlabel('Time step (seconds)')
ylabel('Vehichle State Covariance')
title('Covariance With Odometry Only')
hold on


