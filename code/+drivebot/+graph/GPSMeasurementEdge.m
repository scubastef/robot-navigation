classdef GPSMeasurementEdge < g2o.core.BaseUnaryEdge
   
    properties(Access = protected)
        
        xyOffset;
        
    end
    
    methods(Access = public)
    
        function this = GPSMeasurementEdge(xyOffset)
            this = this@g2o.core.BaseUnaryEdge(2);
            this.xyOffset = xyOffset;
        end
        
        function computeError(this)

	    % Q1d:
        % get the estimated state of the vertex connected with this GPS
        % measurement
        x = this.edgeVertices{1}.estimate();
        psi = g2o.stuff.normalize_theta(x(3));

        c = cos(psi);
        s = sin(psi);

        M = [c -s;
             s c];

        this.errorZ = x(1:2) + (M * this.xyOffset) - this.z;
        
        end
        
        function linearizeOplus(this)

	    % Q1d:
        % partial derivative of errorZ wrt the position
        this.J{1} = [1 0 0;
                     0 1 0];
        end
    end
end
