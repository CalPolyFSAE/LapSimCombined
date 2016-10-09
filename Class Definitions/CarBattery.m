classdef CarBattery < handle
    %UNTITLED8 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Capacity
        Weight
        EffectiveCG
        Resistance
        NominalVoltage
        RoundtripEfficiency

        Name = '';
    end
    
    methods
        function B = CarBattery(Capacity,Weight,CG,Resistance,NominalVoltage,RoundtripEfficiency)
            B.Capacity = Capacity;
            B.Weight = Weight;
            B.EffectiveCG = CG;
            B.Resistance = Resistance;
            B.NominalVoltage = NominalVoltage;
            B.RoundtripEfficiency = RoundtripEfficiency;
        end
        
    end
    
end

