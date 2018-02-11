classdef CarBrakes < handle
    
    properties
        J
        Name = '';
    end
    
    methods
        function B = CarBrakes(J)
            B.J = J;
        end
    end
    
end

