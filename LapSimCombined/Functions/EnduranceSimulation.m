function [ Energy, Time, TF ] = EnduranceSimulation( Car,Track,EnduranceLength,InitialTF )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

InitialGain = 0.05;
SecondOrderGain = 1;
ReverseGain = 0.5;

EnduranceTrack = TestTrack([Track.Track,Track.Track]);
Laps = EnduranceLength/Track.Length;

Gain = InitialGain;
TF = InitialTF;
OldTF = 0;

ConverganceCount = 0;

OldError = 100;

while true
    ConverganceCount = ConverganceCount + 1;
    
    Car.Driveline.SetTorqueFactor(TF);
    
    Tele = Simulate(Car,EnduranceTrack);
    FirstLapP = Tele.LapData(1:Track.Length,8)*0.000112985;
    FirstLapT = Tele.LapData(1:Track.Length,11);
    FirstLapE = sum(FirstLapP.*FirstLapT)/3600;
    FirstLapT = sum(FirstLapT);
    
    SecondLapP = Tele.LapData(Track.Length+1:end,8)*0.000112985;
    SecondLapT = Tele.LapData(Track.Length+1:end,11);
    SecondLapE = sum(SecondLapP.*SecondLapT)/3600;
    SecondLapT = sum(FirstLapT);
    
    Energy = FirstLapE + SecondLapE*(Laps-1);
    Time = FirstLapT + SecondLapT*(Laps-1);
    
    Error = Car.Battery.Capacity - Energy;
    ErrorTracker(ConverganceCount) = Error;
    TFTracker(ConverganceCount) = TF;
    
    
    if abs(Error) < Car.Battery.Capacity/100
        break
    end
    
    
    %     Adjustment = 0.5*(TF-OldTF);
    %     OldTF = TF;
    %     TF = TF - Adjustment;
    
    TF = TF + Error*Gain;
    
    if TF > 1;
        break
    end
    
    if TF <= 0
        
        Energy = inf;
        Time = inf;
        TF = 0;
        
        break
        
    end
    
    %     figure(1)
    %     plot(ErrorTracker,'ro')
    %     xlabel('Iteration Number')
    %     ylabel('Error (kWh)')
    %     grid on
    %     figure
    %     plot(TFTracker,'ro')
    %     xlabel('Iteration Number')
    %     ylabel('Torque Factor')
    %     grid on
end

end

