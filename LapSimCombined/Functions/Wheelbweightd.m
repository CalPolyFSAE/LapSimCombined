function [ RawResults,PointResults ] = Wheelbweightd( CarFcn, TrackFcn )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here


Length  = [];
WF = []';
CG(:,1) = [];

for m = 1:length(WF)
    CG(end+1,1) = Length*(1-WF(m));
end
    
S1 = 1:length(Length);
S2 = 1:length(CG(:,1));

RawResults = zeros(S1*2,8,S2);
EnduranceLength = 866142; %22km in inches
EnduranceLaps = EnduranceLength/Track.Length;


parfor i = S1
    Car = carFcn();
    Track = TrackFcn();  
    
    Wheelbase = Length(i);
    Car.Chassis.Length = Wheelbase;
    
    Tele = Simulate(Car,Track);
    
    TimeAutoX = sum(cell2mat(Tele.Results(1)));
    Time75 = cell2mat(Tele.Results(4));
    MaxG = Car.Tire.MaxLateralAcceleration;
    TimeSkid = 2*pi*sqrt(9.1/(9.81*MaxG));
    
   for j = S2
        
       RPM = RPMCutOffs(j) * GR;
        
        Car.Driveline.SetRPMLimit(RPM);
        
        [Energy, EndTime, TF ] = Simulate(Car,Track,Endurance);

        RawResults(i,:,j) = [TimeAutoX,Time75,TimeSkid,EndTime,Energy,TF,RPM,GR]; 
    
end

