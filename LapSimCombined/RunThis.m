%% Workings of the Combined Lapsimulator for Formula SAE Combustion and Electric
% This is the Master.
% User inputs will mainly be inputted into SetupSheets.xlsx. Toggle which
% car to run through the Lap Simulator here. Dynamic event results as well
% as points will be outputted. 

clear all
clc

% winopen('SetupSheets.xlsx'); %Make sure to save excel before running, you can leave it open
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%%%%%%%
%%%%%%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%%%%%%%
%setup = input(' Combustion or Electric? ','s');
% setup = 'Combustion';
% setup = 'Electric';
%%%%%%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%%%%%%%
%%%%%%%%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize car
rowNumber = 6; 
Car = @()(CarBuilderSS('Electric', rowNumber));
Track = @FSAELincoln2013;
car = Car();
track = Track();
Tele = Simulate(car,track);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find Continuous Current for ESF/FMEA
BatteryCurrent = Tele.LapDataStructure.BatteryCurrent;
BatteryVoltage = Tele.LapDataStructure.BatteryVoltage;
BatteryPower = Tele.LapDataStructure.BatteryPower;          % Watts
MotorTorque = Tele.LapDataStructure.MotorTorque;            % in*lb
MotorRPM = Tele.LapDataStructure.MotorRPM;                  % RPM
MotorPower = Tele.LapDataStructure.MotorPower;              % Watts
Position = Tele.LapDataStructure.Position;
Time = Tele.LapDataStructure.Time;
figure(1)
plot(Time, MotorPower)
xlabel('Time (?)')
ylabel('Motor Power (Watts)')

figure(2)
plot(Time, BatteryCurrent);
xlabel('Time(s)')
ylabel('Motor Current (Amps)')

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% 
% [Results, RawResults] = ExcelSweep(Track,6,15,'Electric');
% 
% resulttimes = Results(:,1);
% endtimes = Results(:,4);
% 
% subplot(2,1,1)
% plot(weightdist, resulttimes); 
% xlabel('Weight Distribution (% Front)'); ylabel('Lap Time (s)');
% subplot(2,1,2)
% plot(weightdist, endtimes);
% xlabel('Weight Distribution (% Front)'); ylabel('Lap Time (s)');


%[ RawResults, Results ] = RPMLimitingAnalysis( Car,Track );
%Simulate( Car,Track )
%save('BatteryandRPMLimitingAnalysis')
