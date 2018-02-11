function [ OutTel, Results ] = GearRatioSweep(Car,Track)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

GearRatios = (2:0.5:5);
%GearRatios = 2;

S = length(GearRatios);

Results = zeros(S,5);

for i = 1:S
    
    GR = GearRatios(i);
    Car.Driveline.GearRatio = GR;
    
    Tele = Simulate(Car,Track);
    
    Time = cell2mat(Tele.Results(1));
    Time = sum(Time);

    AccTime = cell2mat(Tele.Results(4));
    
    Energy = cell2mat(Tele.Results(7));
    
    Score = cell2mat(Tele.Results(9));
    
    Results(i,:) = [GR,Time,AccTime,Energy,Score];
    
    OutTel(i) = Tele;
    
    disp('=====================================================================')
end

figure
plot(Results(:,1),Results(:,2),'ro');
grid on
xlabel('Gear Ratio')
ylabel('Lap Time (s)')

figure
plot(Results(:,1),Results(:,4),'ro');
grid on
xlabel('Gear Ratio')
ylabel('Energy Consumption (kWh)')

figure
plot(Results(:,1),Results(:,3),'ro');
grid on
xlabel('Gear Ratio')
ylabel('75m Straight Time (s)')

figure
plot(Results(:,1),Results(:,5),'ro');
grid on
xlabel('Gear Ratio')
ylabel('Total Score')

disp('=====================================================================')
disp('=====================================================================')
disp('=====================================================================')

end

