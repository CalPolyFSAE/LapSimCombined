function [ Results,TFResults ] = EnduranceAnalysis( Car,Track )
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

tic
Simulate(Car,Track);
Time = toc;

GearRatios = (4.2:0.1:4.7);

S1 = length(GearRatios);

TorqueFactor = (1:-0.005:0.5);

S2 = length(TorqueFactor);

Results = zeros(S1,5);

TFResults = zeros(S1,1);

EstimatedTime = Time*S1*S2/60;

disp(['Estimated minimum run time: ',num2str(EstimatedTime),' minutes'])


for i = 1:S1
    
    NewCar = Car;
    
    GR = GearRatios(i)
    NewCar.Driveline.GearRatio = GR;
        
    for j = 1:S2

        TF = TorqueFactor(j);
        disp(TF)
        NewCar.Motor.OutputCurve(:,2) = NewCar.Motor.OutputCurve(:,2)*TF;
        Tele = Simulate(NewCar,Track);

        Energy = cell2mat(Tele.Results(7));
        Energy = Energy*16.4;

        if Energy < 6.3

            LapTime = cell2mat(Tele.Results(1));

            EnduranceTime = sum(LapTime)*16.4;

            Tmin = 1367.38;
            Tmax = 1.333*Tmin;

            EndScore = 300*(((Tmax/EnduranceTime) - 1)/((Tmax/Tmin) - 1)) + 25;
            if EndScore > 325
                EndScore = 325;
            elseif EndScore < 0
                EndScore = 0;
            end

            Emin = .216;
            EF = (75.97/(EnduranceTime/16.4))*(Emin/(Energy/16.4))^2;
            EFmin = .22;
            EScore = 100*((EFmin/EF)-1)/((EFmin/0.88)-1);
            if EScore > 100
                EScore = 100;
            elseif EScore < 0
                EScore = 0;
            end

            NewCar.Motor.OutputCurve(:,2) = NewCar.Motor.OutputCurve(:,2)/TF;

            Tele = Simulate(NewCar,Track);

            Score = cell2mat(Tele.Results(9));

            TotalScore = Score + EndScore + EScore;

            Results(i,:) = [TotalScore,Score,EndScore,EScore,GR];

            TFResults(i) = TF;

            break

        end

        NewCar.Motor.OutputCurve(:,2) = NewCar.Motor.OutputCurve(:,2)/TF;

    end
    
end

figure
plot(GearRatios,Results(:,1),'ro')
grid on
xlabel('Gear Ratio')
ylabel('Total Score')
CarName = Car.Name;
title([CarName,' Competition Score'])
        
        

end

