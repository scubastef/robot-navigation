% This script runs Q2(c)

% Create the configuration object.
configuration = drivebot.SimulatorConfiguration();

% Enable the laser to support pure SLAM
configuration.enableLaser = true;

% If you set this parameter to false, the simulator generates measurements
% with no noise in them. You might find this useful for debugging.
% However, unless specified otherwise, any submitted results must have this
% value set to true.
configuration.perturbWithNoise = true;

% Set up the simulator
simulator = drivebot.DriveBotSimulator(configuration, 'q2_c');

% Create the localization system
drivebotSLAMSystem = drivebot.DriveBotSLAMSystem(configuration);
drivebotSLAMSystem.setRecommendOptimizationPeriod(inf);

% This tells the SLAM system to do a very detailed check that the input
% appears to be correct but can make the code run slowly. Once you are
% confident your code is working safely, you can set this to false.
drivebotSLAMSystem.setValidateGraph(false);

% Run the main loop and correct results
results = minislam.mainLoop(simulator, drivebotSLAMSystem);

% Minimal output plots. For your answers, please provide titles and label
% the axes.

% Plot optimisation times
minislam.graphics.FigureManager.getFigure('Optimization times');
clf
plot(results{1}.optimizationTimes, '*')
hold on

% Plot the error curves
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleStateHistory')

% Plot covariance
minislam.graphics.FigureManager.getFigure('Vehicle Covariances');
clf
plot(results{1}.vehicleCovarianceHistory')
hold on

% Plot errors
minislam.graphics.FigureManager.getFigure('Errors');
clf
plot(results{1}.vehicleStateHistory'-results{1}.vehicleTrueStateHistory')
hold on

% Plot chi2 values
minislam.graphics.FigureManager.getFigure('chi2 values');
clf
plot(results{1}.chi2Time, results{1}.chi2History)
hold on


% This is how to extract the graph from the optimizer
graph = drivebotSLAMSystem.optimizer();

% This is how to extract cell arrays of the vertices and edges from the
% graph
allVertices = graph.vertices();
allEdges = graph.edges();

% Work out the number of vehicle poses and landmarks. 
numVehicleVertices = 0;
numLandmarks = 0;

landmarkObservationsPerVehicleVertex = 0;
observationsPerLandmarkVertex = 0;

% Q2c:
% Finish implementing the code to capture information about the graph
% structure.

n = 0; % total number of (landmark) observations 

for i=1:length(allVertices)
    % count vehicle vertices
    isVehicleVertex = isa(allVertices{i}, 'drivebot.graph.VehicleStateVertex');
    numVehicleVertices = numVehicleVertices + isVehicleVertex;
    
    % count landmark vertices
    isLandmarkVertex = isa(allVertices{i}, 'drivebot.graph.LandmarkStateVertex');
    numLandmarks = numLandmarks + isLandmarkVertex;
    
    % count landmark observations at vehicle vertices; same as the number
    % of observations made at each time step since the only obsrvation type
    % in this setting is landmark observations.
    if(isVehicleVertex)
        for j=1:length(allVertices{i}.edges)
            edge = allVertices{i}.edges{j};
            n = n + isa(edge, 'drivebot.graph.LandmarkRangeBearingEdge');
        end
    end
    
end

% use the total number of (lanmark) obsrvations, number of vehicle vertices, 
% and number of landmarks to compute quantites of interest
landmarkObservationsPerVehicleVertex = n/numVehicleVertices;
observationsPerLandmarkVertex = n/numLandmarks;

% display the values
disp(['Numer of vehicle poses stored: '  num2str(numVehicleVertices)])
disp(['Number of landmarks initialized: '  num2str(numLandmarks)])
disp(['Average number of observations made by a robot at each timestep: '...
     num2str(landmarkObservationsPerVehicleVertex)])
disp(['Average number of observations recieved by each landmark: '...
     num2str(observationsPerLandmarkVertex)])

% end Qc

