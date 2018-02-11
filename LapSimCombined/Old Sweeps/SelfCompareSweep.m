function [ RawResults,PointResults ] = SelfCompareSweep( Car,Track )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

GearRatios = (3.5:0.1:5);

S1 = length(GearRatios);

RawResults = zeros(S1*2,7);

EnduranceLength = 866142;

EnduranceLaps = EnduranceLength/Track.Length;

h = waitbar(0,'0%','Name','Calculating Round 1...',...
            'CreateCancelBtn',...
            'setappdata(gcbf,''canceling'',1)');
setappdata(h,'canceling',0)

TF = 1;

for i = 1:S1
    
    if getappdata(h,'canceling')
        delete(h)
        return
    end
    
    NewCar = Car;
    
    GR = GearRatios(i);
    NewCar.Driveline.GearRatio = GR;
    
    Tele = Simulate(Car,Track);
    
    TimeAutoX = sum(cell2mat(Tele.Results(1)));
    Time75 = cell2mat(Tele.Results(4));
    MaxG = NewCar.Tire.MaxLateralAcceleration;
    TimeSkid = 2*pi*sqrt(9.1/(9.81*MaxG));
    
    
    [Energy, EndTime, TF ] = EnduranceSimulation(Car,Track,EnduranceLength,TF);
   
    RawResults(i,:) = [TimeAutoX,Time75,TimeSkid,EndTime,Energy,TF,GR];
    
    if TF > 1;
        TF = 1;
    end
    
    waitbar(i/S1,h,sprintf('%4.2f%%',i/S1))
    
end

delete(h)

Car.Weight = Car.Weight - 38;
Car.Battery.Capacity = 4.73;

h = waitbar(0,'0%','Name','Calculating Round 2...',...
            'CreateCancelBtn',...
            'setappdata(gcbf,''canceling'',1)');
setappdata(h,'canceling',0)

TF = 1;

for i = S1+1:S1*2
    
    if getappdata(h,'canceling')
        delete(h)
        return
    end
    
    NewCar = Car;
    
    GR = GearRatios(i-S1);
    NewCar.Driveline.GearRatio = GR;
    
    Tele = Simulate(Car,Track);
    
    TimeAutoX = sum(cell2mat(Tele.Results(1)));
    Time75 = cell2mat(Tele.Results(4));
    MaxG = NewCar.Tire.MaxLateralAcceleration;
    TimeSkid = 2*pi*sqrt(9.1/(9.81*MaxG));
    
    
    [Energy, EndTime, TF ] = EnduranceSimulation(Car,Track,EnduranceLength,TF);
   
    RawResults(i,:) = [TimeAutoX,Time75,TimeSkid,EndTime,Energy,TF,GR];
    
    if TF > 1
        TF = 1;
    end
    
    waitbar((i-S1)/S1,h,sprintf('%4.2f%%',(i-S1)/S1))
    
end

delete(h)


LapTime = RawResults(:,4)/EnduranceLaps;
LapEnergy = RawResults(:,5)/EnduranceLaps;

EFArray = (min(LapTime)./LapTime).*(min(LapEnergy)./LapEnergy).^2;

PointResults = zeros(S1*2,6);

MinTimes = [77.664,3.506,4.901,1367.38,1367.38/EnduranceLaps];

for i = 1:S1*2
    
    %PointResults(i,:) = PointCalculator([min(RawResults(:,1:4)),min(LapTime)],min(LapEnergy),min(EFArray),[RawResults(i,1:4),LapTime(i)],LapEnergy(i));
    PointResults(i,:) = PointCalculator(MinTimes,0.216,0.22,[RawResults(i,1:4),LapTime(i)],LapEnergy(i));

end

plot(RawResults(1:S1,end),PointResults(1:S1,end),'ro')
hold on
plot(RawResults(S1+1:2*S1,end),PointResults(S1+1:2*S1,end),'bo')
grid on
xlabel('Gear Ratio')
ylabel('Competition Score')

end

