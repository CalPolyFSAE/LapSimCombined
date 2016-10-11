classdef CarSuspension < handle
    
    properties
        LinearSpring
        ARB
        UnsprungWeight
        UnsprungHeight
        RollCenters
        PitchCenter
        Name = '';
    end
    
    methods
        function S = CarSuspension(SpringRate,ARBRate,UnsprungW,UnsprungH,RC,PC)
            S.LinearSpring = SpringRate;
            S.ARB = ARBRate;
            S.UnsprungWeight = UnsprungW;
            S.UnsprungHeight = UnsprungH;
            S.RollCenters = RC;
            S.PitchCenter = PC;
        end
        
    end
    
end

