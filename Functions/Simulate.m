function [ Tele ] = Simulate( CarObject,TrackObject )
% Lap Simulation Function
%
% This function simulates a lap around the given track object by the given
% car object.  In order to optimize for speed, look up tables for various
% velocity related parameters are generated and are then used to create
% actual acceleration and braking curves for each track section.  This
% acceleration and braking curves are then stitched together to develop
% brake points and section times for the entire track.
%
% INPUTS
% Name          Type          Units   Description            
%**************************************************************
%
%
% OUTPUTS
% Name          Type          Units   Description            
%**************************************************************
% VARIABLES
% Name          Type          Units   Description            
%**************************************************************
% NONE
%
% FUNCTIONS
% Name          Location         Description            
%**************************************************************
% NONE

dx = 1;

CarObject.Tire.CalculateLateralGMap(CarObject, TrackObject);
CarObject.Tire.CalculateLongitudinalGMap(CarObject);

LookUpTable1 = CarObject.StraightAccTableGenerator();

Velocity = LookUpTable1(:,1);
Drag = LookUpTable1(:,2);
MotorE = LookUpTable1(:,6);

LookUpTable2 = CarObject.StraightDecTableGenerator(Velocity,Drag);

StraightThrottle = ThrottleCurve(LookUpTable1,dx);
StraightBrake = BrakeCurve(LookUpTable2,dx);


% Cornering acceleration look up tables generation

S = TrackObject.Sections;

RArray = zeros(S,2);
MaxV = zeros(S,1);
EntranceV = zeros(S,1);
ExitV = zeros(S,1);

for i = 1:S
    R = TrackObject.Track(i).Radius;
    
    RArray(i,:) = [R,i];
    
    if R
        AccTable = CarObject.CornerAccTableGenerator(R,Velocity,Drag);
        DecTable = CarObject.CornerDecTableGenerator(R,Velocity,Drag);
        TrackObject.Track(i).AccTable = AccTable;
        TrackObject.Track(i).DecTable = DecTable;
        if i == 1
            TrackObject.Track(i).AccCurve = ThrottleCurve( AccTable,dx);
            TrackObject.Track(i).DecCurve = BrakeCurve(DecTable,dx);
        else
            I = find(R == RArray(1:i-1,1),1,'first');
            if I
                j = RArray(I,2);
                TrackObject.Track(i).AccCurve = TrackObject.Track(j).AccCurve;
                TrackObject.Track(i).DecCurve = TrackObject.Track(j).DecCurve;
            else
                TrackObject.Track(i).AccCurve = ThrottleCurve( AccTable,dx);
                TrackObject.Track(i).DecCurve = BrakeCurve(DecTable,dx);
            end
        end
    else
        TrackObject.Track(i).AccCurve = StraightThrottle;
        TrackObject.Track(i).AccTable = LookUpTable1;
        TrackObject.Track(i).DecCurve = StraightBrake;
        TrackObject.Track(i).DecTable = LookUpTable2;
    end
    
    MaxV(i) = TrackObject.Track(i).AccCurve(end,2);
    if i == 1
        EntranceV(1) = 0;
    elseif i == S
        ExitV(i) = MaxV(i);
        ExitV(i - 1) = MaxV(i);
        EntranceV(i) = MaxV(i);
    else
        ExitV(i - 1) = MaxV(i);
        EntranceV(i) = MaxV(i);
    end    
end

[ EntranceV, ExitV, BP, BPSpeed ] = BrakePointIterator( TrackObject,MaxV,EntranceV,ExitV );

Miscellaneous = {EntranceV,ExitV,BP,BPSpeed};
Tele = Telemetry(Miscellaneous);
Tele.LapStitch(TrackObject);
Tele.LapResultCalculator(TrackObject,CarObject.Tire.MaxLateralAcceleration);

end

function [ Results ] = BrakeCurve( Table,dx )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

Vel = Table(:,1);
ARPM = Table(:,3);
MRPM = Table(:,4);
Acc = Table(:,6);
Torque = Table(:,5);
LatGs = Table(:,7);

Vel = Vel.^2;
Acc = -Acc*32.174*12;

StartV = Table(end,1);

M1 = (Acc(2:end) - Acc(1:end-1))./(Vel(2:end) - Vel(1:end-1));
B1 = Acc(2:end) - M1.*Vel(2:end);

M2 = (ARPM(2:end) - ARPM(1:end-1))./(Vel(2:end) - Vel(1:end-1));
B2 = ARPM(2:end) - M2.*Vel(2:end);

M3 = (MRPM(2:end) - MRPM(1:end-1))./(Vel(2:end) - Vel(1:end-1));
B3 = MRPM(2:end) - M3.*Vel(2:end);

M4 = (Torque(2:end) - Torque(1:end-1))./(Vel(2:end) - Vel(1:end-1));
B4 = Torque(2:end) - M4.*Vel(2:end);

M5 = (LatGs(2:end) - LatGs(1:end-1))./(Vel(2:end) - Vel(1:end-1));
B5 = LatGs(2:end) - M5.*Vel(2:end);

V = StartV^2;
X = 0;
A = Acc(end);
AR = ARPM(end);
MR = MRPM(end);
TQ = Torque(end);
LG = LatGs(end);

Result = [X,V,A,AR,MR,TQ,LG];

Results = zeros(6000,7);
TL = zeros(6000,1);

Results(1,:) = Result;
TL(1) = Table(end,8);

i = 1;

while V
    i = i + 1;
    
    a = find(Vel >= V, 1, 'first');
    
    TL(i) = Table(a,end);
    
    if a ~= 1
        a = a - 1;
    end
    
    A = M1(a)*V + B1(a);
    
    oldV = V;
    
    X = (i - 1)*dx;
    V = V + 2*A*dx;
    
    if V < 0
        V = 0;
    end
    
    avgV = (oldV + V)/2;
    
    a = find(Vel >= avgV, 1, 'first');
    
    if a ~= 1
        a = a - 1;
    end

    if a
        AR = M2(a)*avgV + B2(a);
        MR = M3(a)*avgV + B3(a);
        TQ = M4(a)*avgV + B4(a);
        LG = M5(a)*avgV + B5(a);
    else
        AR = M2(end)*avgV + B2(end);
        MR = M3(end)*avgV + B3(end);
        TQ = M4(end)*avgV + B4(end);
        LG = M5(end)*avgV + B5(end);
    end
    
    Result = [X,V,A,AR,MR,TQ,LG];
    
    Results(i,:) = Result;
    
end

Results(:,2) = sqrt(Results(:,2));
Results(:,3) = Results(:,3)/(12*32.174);

I = find(Results(2:end,1) == 0);
I = I + 1;
Results(I,:) = [];
TL(I) = [];

V1 = Results(1:end-1,2);
V2 = Results(2:end,2);

Vavg = (V2 + V1)/2;

T = dx./Vavg;

T = [0;T];

ZeroApp = zeros(length(T),1);

Results = [Results(:,1),Results(:,2),Results(:,3),Results(:,7),...
    TL,Results(:,4),Results(:,5),ZeroApp,ZeroApp,...
    Results(:,6),T];


end

function [ Results ] = ThrottleCurve( Table,dx )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

StopSpeedPercentage = 1;

Vel = Table(:,1);
ARPM = Table(:,3);
MRPM = Table(:,4);
Pow = Table(:,7);
Acc = Table(:,8);
Torque = Table(:,5);
LatGs = Table(:,9);

Vel = Vel.^2;
Acc = Acc*32.174*12;


M1 = (Pow(2:end) - Pow(1:end-1))./(Vel(2:end) - Vel(1:end-1));
B1 = Pow(2:end) - M1.*Vel(2:end);

M2 = (Acc(2:end) - Acc(1:end-1))./(Vel(2:end) - Vel(1:end-1));
B2 = Acc(2:end) - M2.*Vel(2:end);

M3 = (ARPM(2:end) - ARPM(1:end-1))./(Vel(2:end) - Vel(1:end-1));
B3 = ARPM(2:end) - M3.*Vel(2:end);

M4 = (MRPM(2:end) - MRPM(1:end-1))./(Vel(2:end) - Vel(1:end-1));
B4 = MRPM(2:end) - M4.*Vel(2:end);

M5 = (Torque(2:end) - Torque(1:end-1))./(Vel(2:end) - Vel(1:end-1));
B5 = Torque(2:end) - M5.*Vel(2:end);

M6 = (LatGs(2:end) - LatGs(1:end-1))./(Vel(2:end) - Vel(1:end-1));
B6 = LatGs(2:end) - M6.*Vel(2:end);

I = find(Acc < 0, 1, 'first');

if I
    TopSpeed = Vel(I);
else
    TopSpeed = Vel(end);
end

StopSpeed = sqrt(StopSpeedPercentage)*TopSpeed;

V = 0;
X = 0;
A = Acc(1);
P = Pow(1);
AR = ARPM(1);
MR = MRPM(1);
TQ = Torque(1);
LG = LatGs(1);

Result = [X,V,A,P,AR,MR,TQ,LG];

Results = zeros(6000,8);
TL = zeros(6000,1);

Results(1,:) = Result;
TL(1) = Table(1,10);

i = 1;

while V <= StopSpeed
    
    i = i + 1;
    
    a = find(Vel >= V, 1, 'first');
    
    TL(i) = Table(a,end);
    
    if a ~= 1
        a = a - 1;
    end
    
    A = M2(a)*V + B2(a);
    
    oldV = V;
    
    X = (i - 1)*dx;
    V = V + 2*A*dx;
    
    avgV = (oldV + V)/2;
    
    a = find(Vel >= avgV, 1, 'first');
    
    if a ~= 1
        a = a - 1;
    end

    if a
        P = M1(a)*avgV + B1(a);
        AR = M3(a)*avgV + B3(a);
        MR = M4(a)*avgV + B4(a);
        TQ = M5(a)*avgV + B5(a);
        LG = M6(a)*avgV + B6(a);
    else
        P = M1(end)*avgV + B1(end);
        AR = M3(end)*avgV + B3(end);
        MR = M4(end)*avgV + B4(end);
        TQ = M5(end)*avgV + B5(end);
        LG = M6(end)*avgV + B6(end);
    end
    
    Result = [X,V,A,P,AR,MR,TQ,LG];
    
    Results(i,:) = Result;
    
    if X > 500*12
        break
    end
    
end

Results(:,2) = sqrt(Results(:,2));
Results(:,3) = Results(:,3)/(12*32.174);

I = find(Results(2:end,2) == 0);
I = I + 1;

Results(I,:) = [];
TL(I) = [];

Results(end,2) = sqrt(StopSpeed);

V1 = Results(1:end-1,2);
V2 = Results(2:end,2);

Vavg = (V2 + V1)/2;

T = dx./Vavg;

T = [0;T];

ZeroApp = zeros(length(Results),1);

Results = [Results(:,1),Results(:,2),Results(:,3),Results(:,8),...
    TL,Results(:,5),Results(:,6),Results(:,4),Results(:,7),...
    ZeroApp,T];

I = find(Results(:,3) <= 0, 1, 'first');
I = I + 1;

Results(I:end,:) = [];



end

function [ EntranceV, ExitV, BP,BPSpeed ] = BrakePointIterator( Track,MaxV,EntranceV,ExitV )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


S = Track.Sections;
BP = zeros(S,2);
BPSpeed = zeros(S,1);
ICount = 0;
Check = 1;
NegFlag = 1;
while true
    ICount = ICount + 1;
    %disp(['Brake Point Iteration Number: ',num2str(ICount)])
    
    for i = 1:S
    
        [BP(i,1),AdjEntV,AdjExV] = BrakePoint(Track.Track(i),EntranceV(i),MaxV(i),ExitV(i));
        BP(i,2) = Track.Track(i).Length;
        
        ExitV(i) = AdjExV;
        EntranceV(i+1) = ExitV(i);
        
        EntranceV(i) = AdjEntV;
        if i > 1
            ExitV(i-1) = AdjEntV;
        end
        
%         Name = ['SAE Sim 3/Plots/Iteration ', num2str(ICount),'/Section ',num2str(i),'.jpg'];
%         h = gcf;
%         saveas(h,Name);
        
    end
    
    I = find(BP(:,1) < 0);
    
    if I
        NegFlag = 1;
    else
        NegFlag = 0;
    end
    
    if ICount > 100
        disp('Iteration Unstable, Exiting')
        break
    end
    
    if (Check < 0.0001 && ~NegFlag)
        break
    end
    
    Difference = EntranceV(2:S) - ExitV(1:S-1);
    Check = max(Difference);
    
    
        
end
    
end

function [ BP, AdjEntV, AdjExV  ]  = BrakePoint( Section, EntrySpeed, MaxSectionSpeed, TargetExitSpeed )
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here


SectionLength = Section.Length;

Acc = [Section.AccCurve(:,1), Section.AccCurve(:,2)];
Dec = [Section.DecCurve(:,1), Section.DecCurve(:,2)];
%dx = Acc(2,1) - Acc(1,1);
dx=1;

% Shifts data arrays so they coincide with entry and exit speed
AccIndex = find(Acc(:,2) <= EntrySpeed, 1, 'last');
DecIndex = find(Dec(:,2) <= TargetExitSpeed, 1, 'first');

if AccIndex > 1
    adl = Acc(AccIndex,1);
    Acc(:,1) = Acc(:,1) - adl;
end

if Dec(DecIndex,1) ~= SectionLength
    ddl = SectionLength - Dec(DecIndex,1);
    ddl = dx*(round(ddl/dx));
    Dec(:,1) = Dec(:,1) + ddl;
end

% Extends lengths of arrays to match dimensions
% Extend lower bounds
if Acc(1,1) > Dec(1,1)
    AppA = transpose(Dec(1,1):dx:Acc(1,1)-dx);
    AppB = zeros(length(AppA),1);
    Acc = cat(1,[AppA,AppB],Acc);
elseif Acc(1,1) < Dec(1,1)
    AppA = transpose(Acc(1,1):dx:Dec(1,1)-dx);
    AppB = ones(length(AppA),1)*Dec(1,2);
    Dec = cat(1,[AppA,AppB],Dec);
end

% Extend upper bounds
if Acc(end,1) > Dec(end,1)
    AppA = transpose(Dec(end,1)+dx:dx:Acc(end,1));
    AppB = zeros(length(AppA),1);
    Dec = cat(1,Dec,[AppA,AppB]);
elseif Acc(end,1) < Dec(end,1)
    AppA = transpose(Acc(end,1)+dx:dx:Dec(end,1));
    AppB = ones(length(AppA),1)*Acc(end,2);
    Acc = cat(1,Acc,[AppA,AppB]);
end

r = Acc(:,2) > MaxSectionSpeed;  % This can probably be removed
Acc(r,2) = MaxSectionSpeed;

Difference = [Acc(:,1),Acc(:,2) - Dec(:,2)];

NegIndex = find(Difference(:,2) < 0, 1, 'last'); 
PosIndex = find(Difference(:,2) > 0, 1, 'first');
ExIndex =  find(Difference(:,2) == 0, 1, 'last');

if ExIndex
    BP = Difference(ExIndex,1);
    BPSpeed = Acc(ExIndex,2);
elseif PosIndex - NegIndex > 1 % Something fishy here
    BP = Difference(PosIndex,1);
    BPSpeed = Acc(PosIndex,2);
else
    BP = Difference(PosIndex-1,1);
    BPSpeed = Acc(PosIndex-1,2);
end

AccIndex = find(Acc(:,1) <= Section.Length, 1, 'last');

if Acc(AccIndex,2) < (TargetExitSpeed - 0.01)
    AdjExV = Acc(AccIndex,2);
else
    AdjExV = TargetExitSpeed;
end

if BP < 0
    DecIndex = find(Dec(:,1) >= 0, 1, 'first');
    AdjEntV = Dec(DecIndex,2);
else
    AdjEntV = EntrySpeed;
end


% R = Section.Radius;
% plot(Acc(:,1),Acc(:,2),Dec(:,1),Dec(:,2))
% xlabel('Distance (in)')
% ylabel('Velocity (in/s)')
% legend('Acceleration Curve','Braking Curve')
% title(['Section Brake Point Radius: ',num2str(R)])
% hold on
% line([0 0], [0 Dec(1,2)],'Color','b','LineWidth',2);
% line([SectionLength SectionLength], [0 Dec(1,2)],'Color','b','LineWidth',2);
% line([BP BP], [0 Dec(1,2)],'Color','r');
% grid on
% if BP > Section.Length
%    xmax = 1.25*BP;
% else
%    xmax = Section.Length;
% end
% axis([0 xmax 0 BPSpeed*1.5])
% hold off




end



