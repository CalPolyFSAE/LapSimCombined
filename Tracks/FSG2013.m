function [ Track ] = FSG2013( )

SectionMatrix = [
    571,268.080000000000;
    177,0;
    399,261.360000000000;
    1416,0;
    384,354.720000000000;
    398,246.480000000000;
    2349,0;
    257,486.360000000000;
    459,0;
    684,610.320000000000;
    145,0;
    1246,570.960000000000;
    567,272.520000000000;
    991,0;
    534,520.680000000000;
    2064,4983.48000000000;
    291,202.800000000000;
    1863,1907.76000000000;
    490,942.960000000000;
    689,268.200000000000;
    723,281.280000000000;
    358,0;
    558,385.440000000000;
    664,608.880000000000;
    3176,0;
    242,480.840000000000;
    680,1309.68000000000;
    175,0;
    441,193.200000000000;
    3616,0;
    717,952.800000000000;
    729,0;
    430,295.200000000000;
    470,766.320000000000;
    1860,0;
    419,289.920000000000;
    723,0;
    541,758.640000000000;
    838,0;
    358,203.880000000000;
    585,0;
    408,685.680000000000;
    1623,0;
    581,5749.68000000000;
    658,275.280000000000;
    3266,0;
    398,468;
    2277,0;
    331,225.960000000000;
    614,876.240000000000;
    985,0;
    352,775.680000000000;
    569,905.880000000000;
    1480,476.160000000000;
    2198,0;
    278,493.440000000000;
    171,317.520000000000;
    446,1035.24000000000;
    1230,550.200000000000;
    869,0];


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
Track.MinAutoXTime = 75.97; % Should be 77 per results but we use the same course for both.
Track.MinEndLapTime = 75.97;
Track.MinSkidpadTime = 4.714; % Using Michigan because FSG skidpad is done in the wet.
Track.MinAccelerationTime = 3.506;
Track.MinEndEnergy = 0.216;
Track.MaxEndEnergy = 1.123;
Track.MinEndEnergyFactor = 0.1;
Track.MaxEndEnergyFactor = 0.88;

end

