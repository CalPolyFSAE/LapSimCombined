Ws = 480;
Wfus = 70;
Wrus = 70;
FR = [0.45 0.55 ];
Tf = 48;
Tr = 48;
Kf = 200000;
Kr = 152500;
hCG = 8;
hfus = 10.65;
hrus = 10.5;
hrrc = 2;
hfrc = 2.5;

Gs = (0:0.01:2)';

Fz = WeightTransfer( Gs,Ws,Wfus,Wrus,FR,Tf,Tr,Kf,Kr,hCG,hfus,hrus,hfrc,hrrc );

[Fy,SA] = Hoosier13(Fz);

FyFront = Fy(:,1) + Fy(:,2);
FyRear  = Fy(:,3) + Fy(:,4);

W = Ws + Wrus + Wfus;
Wf = W*FR(1);
Wr = W*FR(2);

FrontGs = FyFront/Wf;
RearGs  = FyRear/Wr;

OutGs = zeros(length(FrontGs),1);

I = FrontGs > RearGs;
OutGs(I) = RearGs(I);
I = RearGs >= FrontGs;
OutGs(I) = FrontGs(I);

Difference = OutGs - Gs;
I1 = find(Difference >= 0,1,'last');
I2 = find(Difference < 0, 1,'first');

Diff1 = abs(Difference(I1));
Diff2 = abs(Difference(I2));

if Diff1 > Diff2
    I = I2;
else
    I = I1;
end

disp(RearGs(I))
disp(FrontGs(I))

disp(OutGs(I))

% figure
% plot(Gs,Fz(:,1),Gs,Fz(:,2),Gs,Fz(:,3),Gs,Fz(:,4))
% legend('Inside Front','Outside Front','Inside Rear','Outside Rear')
% xlabel('Acceleration Gs')
% ylabel('Normal Loading (lbf)')
% 
% figure
% plot(Fz(:,1),Coeff(:,1),Fz(:,2),Coeff(:,2),Fz(:,3),Coeff(:,3),Fz(:,4),Coeff(:,4))
% legend('Inside Front','Outside Front','Inside Rear','Outside Rear')
% xlabel('Normal Loading (lbf)')
% ylabel('Max Grip Coefficient')
% 
% figure
% plot(SA(:,1),Fz(:,1),SA(:,2),Fz(:,2),SA(:,3),Fz(:,3),SA(:,4),Fz(:,4))
% legend('Inside Front','Outside Front','Inside Rear','Outside Rear')
% xlabel('Slip Angle of Maximum Grip Coefficient')
% ylabel('Normal Loading (lbf)')
% 
% 
% 
% figure
% plot(Gs)
% hold on
% plot(OutGs,'r')
% hold off