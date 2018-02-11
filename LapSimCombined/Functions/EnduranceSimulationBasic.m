function [ Energy, Time, TF, Tele ] = EnduranceSimulationBasic( Car,Track,EnduranceLength)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

track = Track();
TF = 1;
TFDecrement = .1;

Laps = ceil(EnduranceLength/track.Length);

while true
    Car.Driveline.SetTorqueFactor(TF);
    Tele = Simulate(Car, track);
    
    if ~isreal(sum(Tele.LapDataStructure.Time(1:track.Length)))
        display('Not Real');
    end
    
    FirstLapP = Tele.LapDataStructure.BatteryPower(1:track.Length);
    FirstLapT = Tele.LapDataStructure.Time(1:track.Length);
    FirstLapE = sum(FirstLapP.*FirstLapT)/3600/1000;
    FirstLapT = sum(FirstLapT);
    
%     SecondLapP = Tele.LapDataStructure.BatteryPower(track.Length+1:end);
%     SecondLapT = Tele.LapDataStructure.Time(track.Length+1:end);
%     SecondLapE = sum(SecondLapP.*SecondLapT)/3600/1000;
%     SecondLapT = sum(SecondLapT);
    
    Energy = FirstLapE * Laps;% + SecondLapE*(Laps-1);
    Time = FirstLapT * Laps;% + SecondLapT*(Laps-1);
    
    Error = Car.Battery.Capacity - Energy;
    
    if Error > Car.Battery.Capacity/50
        Time = inf;
        TF = 0;
        break;
    end
    
    TF = TF - TFDecrement;
    
    if TF < .1
        Time = inf;
        TF = 0;
        break;
    end
end

