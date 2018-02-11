function [ Track ] = FSAELincoln2012( )
%FSAEWest2011 constructs a track based on the 2011 FSAE West competition
%track.
%   The defined track matches the race line data generated by the operator
%   of www.fsaesim.com.

% Defines section data, one straight section folloed by several curved
% sections.

SectionMatrix = [ %This is in meters
        37.7,0;
        17.9,30.2;
        13.1,12.4;
        8.9,9.9;
        12.1,15.1;
        10,20;
        12.2,21.1;
        16,188;
        50.8,31.9;
        29.1,0;
        13.3,40.3;
        13.6,17.7;
        16,17.2;
        17.6,19.1;
        15.6,50.8;
        47.4,0;
        21.3,21.4;
        29.6,15.8;
        2.5,0;
        21.4,8.3;
        60,0;
        15.8,24.8;
        16,13.7;
        11.1,12.4;
        12.3,15.9;
        16.1,22.9;
        8.1,15;
        11,0;
        31.9,13.9;
        35.2,39.1;
        13.1,0;
        51.1,25.6;
        23,52;
        32.3,19.5;
        10.1,0;
        12.8,10;
        29.7,28.7;
        30.7,0;
        12.3,17.3;
        18.4,8.8;
        61.6,0;
        19.9,20.5;
        14.2,8.1;
        29.7,22.5;
        17.6,0;
        47.2,17.2;
        25.3,20.9;
        6.3,0;
        16,10.9;
        5.9,0;
        12.8,9.7;
        20,10;
        22.6,18.6;
        13.6,0;
    ];

SectionMatrix = round(SectionMatrix * 39.3701); %Convert to integer inches


N = length(SectionMatrix);

% Builds array of TrackSection objects
for i=1:N
    if SectionMatrix(i,2)
        Length = SectionMatrix(i,1);
        Radius = SectionMatrix(i,2);
        TS = TrackSection(Length,Radius,i);
        TSArray(i) = TS;
    else
        Length = SectionMatrix(i,1);
        TS = TrackSection(Length,0,i);
        TSArray(i) = TS;
    end
end

% Calls TestTrack Constructor
Track = TestTrack(TSArray);


end

