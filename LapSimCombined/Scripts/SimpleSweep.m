% NoRegenRaw = squeeze(RawResults(1,:,:));
% RegenRaw = squeeze(RawResults(2,:,:));
%
% Scores = [];
%
% for i = 1:size(NoRegenRaw,1)
%     for j = 1:size(NoRegenRaw,2)
%         Scores(i,j) = sum(NoRegenRaw{i,j}.Results{9});
%     end
% end
%
% for i = 1:size(RegenRaw,1)
%     for j = 1:size(RegenRaw,2)
%         RegenScores(i,j) = sum(RegenRaw{i,j}.Results{9});
%     end
% end

SweepParameter = round((5000:-250:2000) / 4.16);
SweepLength = length(SweepParameter);

Time = [];
Energy = [];

parfor i=1:SweepLength
    car = CarBuilderSS('Electric', 6);
    track = FSG2013;
    
    car.Driveline.SetRPMLimit(SweepParameter(i));
    Tele = Simulate(car,track);
    Time(i) = sum(cell2mat(Tele.Results(1)));
    
    FirstLapP = Tele.LapDataStructure.BatteryPower(1:track.Length);
    FirstLapT = Tele.LapDataStructure.Time(1:track.Length);
    Energy(i) = sum(FirstLapP.*FirstLapT)/3600/1000;
end

parameterName = 'RPM Limit';
figure(1);
plot(SweepParameter,Time);
xlabel(parameterName);
ylabel('Lap Time(s)');
figure(2);
plot(SweepParameter,Energy);
xlabel(parameterName);
ylabel('Energy (kWh)');
    
    
    