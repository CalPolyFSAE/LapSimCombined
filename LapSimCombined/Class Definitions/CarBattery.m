classdef CarBattery < handle
    %UNTITLED8 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Capacity
        Resistance
        NominalVoltage
        RoundtripEfficiency

        Name = '';
    end
    
    methods
        function B = CarBattery(Capacity,Resistance,NominalVoltage,RoundtripEfficiency)
            B.Capacity = Capacity;
            B.Resistance = Resistance;
            B.NominalVoltage = NominalVoltage;
            B.RoundtripEfficiency = RoundtripEfficiency;
        end
        
    end
    
end

