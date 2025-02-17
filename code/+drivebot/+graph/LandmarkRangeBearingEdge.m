% This edge encodes a 3D range bearing measurement.
%
% The measurement is in spherical polar coordinates

% Jacobian from https://github.com/petercorke/robotics-toolbox-matlab/blob/master/RangeBearingSensor.m

classdef LandmarkRangeBearingEdge < g2o.core.BaseBinaryEdge
    
    methods(Access = public)
    
        function this = LandmarkRangeBearingEdge()
            this = this@g2o.core.BaseBinaryEdge(2);
        end
        
        function initialize(this)
            % Q2b: initialize the landmark state
            % see report for derivation.
            
            % get estimated vehcile positionan heading at this time step
            x = this.edgeVertices{1}.x; 
            
            % infer landmark postion from vehicle position and heading
            x_i = this.z(1)*cos(this.z(2) + x(3)) + x(1);
            y_i = this.z(1)*sin(this.z(2) + x(3)) + x(2);
            
            % set the state estimate of the landmark vertex
            this.edgeVertices{2}.setEstimate([x_i; y_i]);
        end
        
        function computeError(this)

            % Q2b:
            % compute difference between landmark position and estimated vehicle

            % get predicted position at this vehcle state vertex
            x = this.edgeVertices{1}.estimate();

            % get predicted landmark position at this landmark vertex
            landmark = this.edgeVertices{2}.estimate();

            % for convenience, compute difference between vehicle and
            % landmark position
            dx = landmark(1:2) - x(1:2);

            % compute error; making sure to wrap the angle
            this.errorZ(1) = norm(dx) - this.z(1);
            this.errorZ(2) = g2o.stuff.normalize_theta(atan2(dx(2), dx(1)) - x(3) - this.z(2));

        end
        
        function linearizeOplus(this)
            % Q2b:
            % compute Jacobian wrt vehichle state vertex and Jacobian wrt
            % landmark state vertex

            % get estimation of vehicle state and landmark state
            x = this.edgeVertices{1}.estimate();
            landmark = this.edgeVertices{2}.estimate();

            % for convience...
            dx = landmark(1:2) - x(1:2);
            r = norm(dx);
            
            % compute jacobian wrt vehcile state
            this.J{1} = ...
                [-dx(1)/r -dx(2)/r 0;
                dx(2)/r^2 -dx(1)/r^2 -1];
            
            % compute jacobian wrt landmark state
            this.J{2} = [dx(1)/r dx(2)/r;
                         -dx(2)/(r^2) dx(1)/(r^2)];
        end        
    end
end