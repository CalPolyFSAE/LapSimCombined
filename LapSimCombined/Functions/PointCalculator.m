function [ Scores ] = PointCalculator( Tmin,Emin,EFmin,Times,LapEnergy )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

Tmin75 = Tmin(1);
TminAutoX = Tmin(2);
TminSkid = Tmin(3);
TminEnd = Tmin(4);
TminEndLap = Tmin(5);

T75 = Times(1);
TAutoX = Times(2);
TSkid = Times(3);
TEnd = Times(4);
TEndLap = Times(5);

Tmax = Tmin*1.50;
AccScore = (71.5*(Tmax/T75-1))/((Tmax/Tmin75)-1) + 3.5;
if AccScore > 75
    AccScore = 75;
elseif AccScore < 0
    AccScore = 0;
end

Tmax = 1.25*Tmin;
AutoXScore = 95.5*(Tmax/TAutoX - 1)/(Tmax/TminAutoX - 1) + 4.5;
if AutoXScore > 100
    AutoXScore = 100;
elseif AutoXScore < 0
    AutoXScore = 0;
end

Tmax = 1.25*TminSkid;
SkidPadScore = 71.5*((Tmax/TSkid)^2-1)/((Tmax/TminSkid)^2-1) + 3.5;
if SkidPadScore > 75
    SkidPadScore = 75;
elseif SkidPadScore < 0
    SkidPadScore = 0;
end

Tmax = 1.45*TminEnd;
if TEnd > TMax
    Endscore = NumberOfEnduranceLaps;
else
    EndScore = 250*(((Tmax/TEnd) - 1)/((Tmax/TminEnd) - 1)) + 50;
end


EF = (TminEndLap/TEndLap)*(Emin/(LapEnergy))^2;
EScore = 100*((EFmin/EF)-1)/((EFmin/0.88)-1);
if EScore > 100
    EScore = 100;
elseif EScore < 0
    EScore = 0;
end

TotalScore = AccScore + AutoXScore + SkidPadScore + EndScore + EScore;

Scores = [AccScore,AutoXScore,SkidPadScore,EndScore,EScore,TotalScore];

end

