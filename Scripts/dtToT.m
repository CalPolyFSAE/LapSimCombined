function Time=dtToT(dT)
Time = zeros(size(dT));
Time(1) = 0;
for n=2:size(dT)
    Time(n)=Time(n-1)+dT(n);
end
end