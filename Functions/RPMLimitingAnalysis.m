function [ RawResults, PointsResults ] = RPMLimitingAnalysis(CarFcn, TrackFcn)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

% Track = TrackFcn();

RegenOnOff = [0 1];
RegenLength = 2;

GearRatios = (5:-0.1:3);
GearRatioLength = length(GearRatios);

RPMCutOffs = (5500:-500:3000);
RPMCutOffLength = length(RPMCutOffs);

RawResults = cell(GearRatioLength, RPMCutOffLength);
PointsResults = zeros(GearRatioLength, RPMCutOffLength);

EnduranceLength = 866142; % 22km in inches
% EnduranceLaps = EnduranceLength/Track.Length;

parfor j = 1:GearRatioLength
    Car = CarFcn();
    Track = TrackFcn();

    %        Car.CG = [(Car.Chassis.Length * (1-WeightDistribution(i))) Car.CG(2) Car.CG(3)];
    GR = GearRatios(j);
    Car.Driveline.SetGearRatios(GR, Car.Motor.OutputCurve);

    Tele = Simulate(Car,Track);

%     if RegenOnOff(i) == 0
%         Car.BrakingMode = 'Hydraulic';
%     else
%         Car.BrakingMode = 'Regen';
%     end

    for k = 1:RPMCutOffLength
        RPM = round(RPMCutOffs(k) / GR);
        Car.Driveline.SetRPMLimit(RPM);

        [~, ~, ~, TeleEndurance ] = EnduranceSimulationBasic(Car,Track,EnduranceLength);
        
        % Fill in Endurance Times & Scores
        Tele.EnduranceLapDataStructure = TeleEndurance.LapDataStructure;
        Tele.Miscellaneous{4} = TeleEndurance.Miscellaneous{4};  
        Tele.Miscellaneous{5} = TeleEndurance.Miscellaneous{5};
        
        Tele.Results{13} = TeleEndurance.Results{13};
        Tele.Results{14} = TeleEndurance.Results{14};
        Tele.Results{9} = ...
            Tele.Results{10} + ...
            Tele.Results{11} + ...
            Tele.Results{12} + ...
            Tele.Results{13} + ...
            Tele.Results{14};

        RawResults{j,k} = Tele;
        PointsResults(j,k) = Tele.Results{9};
    end
end

% parfor i = S1+1:S1*2
%     Car = CarBuilderSS('Electric', 6);
%     Track = FSAELincoln2013;
%     TF=1;
%
%     Car.Weight = Car.Weight - 38;
%     Car.Battery.Capacity = 4.73;
%     GR = GearRatios(i-S1);
%     Car.Driveline.GearRatio = GR;
%
%     Tele = Simulate(Car,Track);
%
%     TimeAutoX = sum(cell2mat(Tele.Results(1)));
%     Time75 = cell2mat(Tele.Results(4));
%     MaxG = Car.Tire.MaxLateralAcceleration;
%     TimeSkid = 2*pi*sqrt(9.1/(9.81*MaxG));
%
%     AxleOutputCurve = Car.Motor.OutputCurve;
%
%     for j = 1:S2
%
%         RPM = RPMCutOffs(j);
%
%         Car.Motor.OutputCurve(RPM+2:end,:) = [];
%
%         [Energy, EndTime, TF ] = EnduranceSimulation(Car,Track,EnduranceLength,TF);
%
%         RawResults(i,:,j) = [TimeAutoX,Time75,TimeSkid,EndTime,Energy,TF,RPM,GR];
%
%         if TF > 1
%             TF = 1;
%         end
%
%     end
%
%     Car.Motor.OutputCurve = AxleOutputCurve;
%
% end


% LapTime = RawResults(:,4)/EnduranceLaps;
% LapEnergy = RawResults(:,5)/EnduranceLaps;
%
% EFArray = (min(LapTime)./LapTime).*(min(LapEnergy)./LapEnergy).^2;
%
% PointResults = zeros(S1*2,6);
%
% CompMinTimes = [3.950,51.569,4.827,1820.652,1820.652/ceil(EnduranceLaps)];
% OurMinTimes = [min(RawResults(:,1:4)),min(LapTime)];
% OverallMinTimes = min(CompMinTimes, OurMinTimes);
%
%
% for i = 1:S1*2
%     for j = 1:S2
%         PointResults(i,:,j) = PointCalculator(OverallMinTimes,min(LapEnergy),min(EFArray),[RawResults(i,1:4),LapTime(i)],LapEnergy(i));
%     end
%
% end


end


