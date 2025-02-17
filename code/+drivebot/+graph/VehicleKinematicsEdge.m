% This class uses a slightly simpler model for the vehicle kinematics used
% in the lectures. This is the more standard built in type for estimate.
%
% The model assumes that the vehicle speed is specified in the vehicle
% frame and is then projected into the world frame. Specifically,
%
% M = [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
%
% The process model has the form:
%
% x = x + M * [vx;vy;theta]
%
% where vx, vy and vtheta are the velocities.
%
% The error model 
% eTheta = 

classdef VehicleKinematicsEdge < g2o.core.BaseBinaryEdge
    
    properties(Access = protected)
        % The length of the time step
        dT;
    end
    
    methods(Access = public)
        function this = VehicleKinematicsEdge(dT)
            assert(dT >= 0);
            this = this@g2o.core.BaseBinaryEdge(3);            
            this.dT = dT;
        end
       
        function initialize(this)
            
                        
            priorX = this.edgeVertices{1}.x;

            c = cos(priorX(3));
            s = sin(priorX(3));
            
            M = this.dT * [c -s 0;
                s c 0;
                0 0 1];
            
            % Compute the posterior assuming no noise
            this.edgeVertices{2}.x = this.edgeVertices{1}.x + M * this.z;

            % Wrap the heading to -pi to pi
            this.edgeVertices{2}.x(3) = g2o.stuff.normalize_theta(this.edgeVertices{2}.x(3));

        end
        
        function computeError(this)
    
            % Q1b:

            % get the x from previous time step (x_k)
            priorX = this.edgeVertices{1}.x;
            
            % define inverse of M(phi_k)
            c = cos(priorX(3));
            s = sin(priorX(3));

            Minv = [c s 0;
                  -s c 0;
                  0 0 1];
 
            
           
            % get the difference of between current and prior x
            dx = this.edgeVertices{2}.x - priorX;

            % normalize the angle of the difference
            dx(3) = g2o.stuff.normalize_theta(dx(3));

            % compute the error
            this.errorZ = ((Minv/this.dT) * dx) - this.z;
 
            % normalize the heading angle for final solution
            this.errorZ(3) = g2o.stuff.normalize_theta(this.errorZ(3));
            
        end
        
        % Compute the Jacobians
        function linearizeOplus(this)

            % Q1b:
            % get prior x (x_k)
            priorX = this.edgeVertices{1}.x;
            
            % define inverse of M(phi_k)
            c = cos(priorX(3));
            s = sin(priorX(3));

            Minv = [c s 0;
                    -s c 0;
                    0 0 1];
            
            % get difference between x_{k+1}  and x_k
            dx = this.edgeVertices{2}.x - priorX;
            
            % derivative of errorZ wrt x_{k+1}
            this.J{2} = Minv / this.dT;

            % derivative of errorZ wrt x_k
            this.J{1}(1,1) = -c;
            this.J{1}(1,2) = -s;
            this.J{1}(1,3) = -dx(1) * s + dx(2) * c;
            this.J{1}(2,1) = s;
            this.J{1}(2,2) = -c;
            this.J{1}(2,3) = -dx(1) * c - dx(2) * s;
            this.J{1}(3,3) = -1;
            this.J{1} = this.J{1} / this.dT;

        end
    end    
end