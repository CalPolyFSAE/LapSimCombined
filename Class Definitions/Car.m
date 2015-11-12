classdef Car < handle
    %UNTITLED12 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        Brakes
        Driveline
        Motor
        Chassis
        Battery
        Suspension
        Tire
        DragCoefficient
        LiftCoefficient
        CenterOfPressure %in
        FrontCrossSection %in^2
        Rho %slug/ft^3
        Weight
        CG
        SprungMass
        UnsprungMass
        Keq
        Name = '';
    end
    
    methods
        function C = Car(Brakes,Driveline,Motor,Chassis,Battery,Suspension,Tire,DragC,XArea)
            C.Brakes = Brakes;
            C.Driveline = Driveline;
            C.Motor = Motor;
            C.Chassis = Chassis;
            C.Battery = Battery;
            C.Suspension = Suspension;
            C.Tire = Tire;
            C.DragCoefficient = DragC;
            C.FrontCrossSection = XArea;
            
            C.UnsprungMass = Brakes.UnsprungMass + Driveline.UnsprungMass + Suspension.UnsprungMass + Tire.Weight/2;
            C.SprungMass = Battery.Weight + Brakes.SprungMass + Chassis.TotalWeight + Driveline.SprungMass + Motor.Weight + Suspension.SprungMass;
            
            C.Weight = sum(C.UnsprungMass) + C.SprungMass;
            C.CG = (Brakes.Weight.*Brakes.EffectiveCG + Driveline.Weight.*Driveline.EffectiveCG...
                + Motor.Weight.*Motor.EffectiveCG + Chassis.TotalWeight.*Chassis.EffectiveCG...
                + Battery.Weight.*Battery.EffectiveCG + Suspension.Weight.*Suspension.EffectiveCG...
                + Tire.Weight.*Tire.EffectiveCG)/C.Weight;
            
            I = Tire.J + Brakes.J + Driveline.J;
            R = Tire.Radius;
            M = C.Weight/32.174;
            
            C.Keq = (I/(R^2*M)) + 1;

        end      
            
        function [ LookUpTable ] = StraightAccTableGenerator( CarObject )
        
            NMotors = CarObject.Motor.NMotors;

            RHO = CarObject.Rho; %slugs/ft^3

            RollingR = CarObject.Weight*CarObject.Tire.RollingResistance; % Rolling Resistance force for car configuration

            % Pull Motor RPM values from motor torque curve
            MotorRPM = CarObject.Motor.OutputCurve(:,1);
            % Pull Motor Torque values from motor torque curve
            MotorT   = NMotors*CarObject.Motor.OutputCurve(:,2);  %in-lb
            % Pull Motor Efficiency values from motor torque curve
            MotorE   = (CarObject.Motor.OutputCurve(:,3));
            % Calculate Axle RPM for each Motor RPM value
            
  %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
  % Need to incorporate sifting in AxleRPM for CarObject.Driveline.GearRatio          
            AxleRPM  = MotorRPM/CarObject.Driveline.GearRatio;
            % Calculate Car Velocity for each Axle RPM value
            Velocity = CarObject.Tire.Radius*AxleRPM*pi/30; % in/s
            % Calculate corresponding drag for each veloicty
            Drag     = (0.5*RHO*CarObject.DragCoefficient*CarObject.FrontCrossSection.*Velocity.^2)/12^4; % lbf
     %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
     %AxleT gear ratio
            % Caluclate axle torque for each motor torque value
            AxleT    = MotorT*CarObject.Driveline.GearRatio*CarObject.Driveline.Efficiency; % in lbf
            % Calculate force at the wheels for each axle torque value
            WheelF   = AxleT/CarObject.Tire.Radius; % lbf
            % Calculate theoretical acceleration for each wheel force value in in/s^2
            % and Gs
            
   %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
   % Shifting affects WheelF in MotorA
            MotorA   = (WheelF - Drag - RollingR)/(CarObject.Keq*CarObject.Weight/(12*32.174)); % in/s^2
            MotorGs  = MotorA/(12*32.174);

            % Compare acceleration to possible acceleration from tires and reduce
            % accelerations to those values
            maxForwardGsTire = interp1(CarObject.Tire.ForwardAccelerationMap.velocities, CarObject.Tire.ForwardAccelerationMap.accelerations, Velocity, 'spline');
            ForwardGs = max(MotorGs, maxForwardGsTire);
            
   %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%         
           %Shifting affects GearRatio in MotorT
            MotorT = (CarObject.Keq*CarObject.Weight*ForwardGs + Drag + RollingR)*CarObject.Tire.Radius/((CarObject.Driveline.GearRatio)*CarObject.Driveline.Efficiency);
            
            % Calculate power consumption for each motor rpm
            Power    = ((MotorT.*MotorRPM./((MotorE)*pi/30)));
            
            LateralGs = zeros(length(Velocity),1);
            
            % Tractive limit is reached at all of the indexes that were
            % previously adjusted to match tire acceleration
            [~, TractiveLimit] = ismember(ForwardGs, maxForwardGsTire);
                        %    1       2     3        4       5      6      7
            LookUpTable = [Velocity,Drag,AxleRPM,MotorRPM,MotorT,MotorE,Power,ForwardGs,LateralGs,TractiveLimit];
            
            
        end
            
        function [ LookUpTable ] = StraightDecTableGenerator(CarObject,Velocity,Drag)
            RollingR = CarObject.Weight*CarObject.Tire.RollingResistance; % Rolling Resistance force for car configuration

            % Assume brakes use tire at full potential
            ForwardGs = interp1(CarObject.Tire.BrakingAccelerationMap.velocities, CarObject.Tire.BrakingAccelerationMap.accelerations, Velocity, 'spline');
            
            % Calculate wheel force based on tire capability. Includes drag
            % strangely.
            WheelF = CarObject.Keq*CarObject.Weight*ForwardGs - Drag - RollingR; 
            
            % Calculate axle and motor rpms based on velocity
            AxleRPM = Velocity/(CarObject.Tire.Radius*pi/30);
      %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      %Shifting
            MotorRPM = AxleRPM*CarObject.Driveline.GearRatio;
            % Recalculate applied brake torque based on wheel force
            BrakeTorque = WheelF*CarObject.Tire.Radius;
            
            % Straight brake curve, therefore lateral Gs is always zero
            LateralGs = zeros(length(Velocity),1);
            
            % Tractive limit is reached at all of the indexes that were
            % previously adjusted to match tire acceleration
            TractiveLimit = ones(length(ForwardGs),1);
                        %    1       2     3        4       5           6           7
            LookUpTable = [Velocity,Drag,AxleRPM,MotorRPM,BrakeTorque,ForwardGs,LateralGs,TractiveLimit];


        end
        
        function [ LookUpTable ] = CornerAccTableGenerator( CarObject,R,Velocity,Drag,MotorE )
        
            RollingR = CarObject.Weight*CarObject.Tire.RollingResistance; % Rolling Resistance force for car configuration
            
            % Pulls max lateral Gs available from map.
            MaxLatG = interp1(CarObject.Tire.LateralAccelerationMap.radii, CarObject.Tire.LateralAccelerationMap.accelerations, R, 'linear');
            
            % Finds lateral Gs for each velocity in the given array
            LateralGs = (Velocity.^2/R)/(32.174*12);
            % Find indexes where velocity lateral Gs are larger than
            % maximum available from tires
            I = LateralGs > MaxLatG;
            % Trim those velocities from the arrays
            LateralGs(I) = [];
            Velocity(I) = [];
            Drag(I) = [];
            MotorE(I) = [];
            % Find max forward Gs available from tires for each lateral G
            ForwardGs = CarObject.Tire.GGCurve(LateralGs,'Throttle', Velocity);

            % Calculate wheel forces, axle torque, and motor torque based
            % on given forward Gs
            WheelF = ForwardGs*CarObject.Weight*CarObject.Keq + Drag + RollingR;
            AxleT  = WheelF*CarObject.Tire.Radius;
  %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
  %MotorT is affected by shifting thourgh CarObject.Driveline.GearRatio
            
            MotorT = AxleT/((CarObject.Driveline.GearRatio)*CarObject.Driveline.Efficiency);
            % Pull available motor torque
            MotorTrueT = CarObject.Motor.OutputCurve(:,2);
            % Eliminate motor torques that occur at velocities above
            % available lateral Gs
            MotorTrueT(I) = [];
            
            % Find calculated motor torques that are more than the motor is
            % capable of
            I = MotorT > MotorTrueT;
            % And set those values to available motor torque
            MotorT(I) = MotorTrueT(I);
            % Recalculate axle torque, wheel force and forward Gs based on adjusted
            % available torque values
            
  %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
  % AxleT affected by shifting through CarObject.Driveline.GearRatio
            AxleT = MotorT*(CarObject.Driveline.GearRatio)*CarObject.Driveline.Efficiency; % in lbf
            WheelF = AxleT/CarObject.Tire.Radius; % lbf
            ForwardGs = (WheelF - Drag - RollingR)/(CarObject.Keq*CarObject.Weight);
            % calculate axle and motor rpms from given velocity array
            AxleRPM = Velocity*30/(pi*CarObject.Tire.Radius);
            
  %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
  % MotorRPM affected by shifting through CarObject.Driveline.GearRatio
            MotorRPM = AxleRPM*(CarObject.Driveline.GearRatio);
            % calculate power consumption based on motor torque, rpm and
            % efficiency
            Power = ((MotorT.*MotorRPM./MotorE)*pi/30);
            
            % Tractive limit is reached at all indexes not limited by motor
            % torque
            TractiveLimit = ~I;

            LookUpTable = [Velocity,Drag,AxleRPM,MotorRPM,MotorT,MotorE,Power,ForwardGs,LateralGs,TractiveLimit];
            
        end
        
        function [ LookUpTable ] = CornerDecTableGenerator( CarObject,R,Velocity,Drag )
            
            RollingR = CarObject.Weight*CarObject.Tire.RollingResistance; % Rolling Resistance force for car configuration
            
            % Pull max lateral Gs from tire model
            MaxLatA = interp1(CarObject.Tire.LateralAccelerationMap.radii, CarObject.Tire.LateralAccelerationMap.accelerations, R, 'linear');
            
            % Calculate lateral Gs for each velocity in given array
            LateralGs = (Velocity.^2/R)/(32.174*12);
            % Find indexes where lateral Gs at given velocity is more than
            % available from tire model
            I = LateralGs > MaxLatA;
            % and trim those indexes
            LateralGs(I) = [];
            Velocity(I) = [];
            Drag(I) = [];
            % Find maximum possible backward Gs at a given lateral G from
            % tire model
            BackGs = CarObject.Tire.GGCurve(LateralGs,'Brake');
            
            % Calculate wheel force and brake torque based on backward Gs
            WheelF = BackGs*CarObject.Weight*CarObject.Keq - Drag - RollingR;
            BrakeTorque = WheelF*CarObject.Tire.Radius;
            % Find indexes where calculated brake torque is more than
            % available from brakes
            I = find(BrakeTorque > sum(CarObject.Brakes.Torque));
            if I
                % and set brake torque to the limiting available torque
                BrakeTorque(I) = sum(CarObject.Brakes.Torque);
                % Recalculate wheel force and Gs
                WheelF = BrakeTorque/CarObject.Tire.Radius;
                BackGs = (WheelF + Drag + RollingR)/(CarObject.Keq*CarObject.Weight);
            end
            
            % Calculate axle and motor rpms based on given velocity array
            AxleRPM = Velocity/(CarObject.Tire.Radius*pi/30);
            
      %~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
      % MotorRPM affected by CarObject.Driveline.GearRatio      
      MotorRPM = AxleRPM*(CarObject.Driveline.GearRatio);
            
            % Tractive limit is reached at all indexes where braking torque
            % is less than the available braking torque
            TractiveLimit = BrakeTorque < sum(CarObject.Brakes.Torque);
            
            LookUpTable = [Velocity,Drag,AxleRPM,MotorRPM,BrakeTorque,BackGs,LateralGs,TractiveLimit];
            
        end
        
        function [deltaFz] = CalculateAeroEffects(CarObject, Velocity)
            %   Load Distribution Equations from: Solve[{Fz + Rz + Lift == 0, -Fz*frontAxleDistance + Rz*rearAxleDistance - drag*(CoPz - CGz) + lift*(CoPx - CGx) == 0}, {Fz, Rz}]
            
            lift = (0.5*CarObject.Rho*CarObject.LiftCoefficient*CarObject.FrontCrossSection*Velocity.^2)/12^4; % lbf
            drag = (0.5*CarObject.Rho*CarObject.DragCoefficient*CarObject.FrontCrossSection*Velocity.^2)/12^4; % lbf
            
            frontAxleDistance = CarObject.CG(1);
            rearAxleDistance = CarObject.Chassis.Length - CarObject.CG(1);
            
            CGx = CarObject.CG(1);
            CGz = CarObject.CG(3);
            CoPx = CarObject.CenterOfPressure(1);
            CoPz = CarObject.CenterOfPressure(3);
            
            Fy_diff = -1 * ((-1*CGz*drag + CoPz*drag + CGx*lift - CoPx*lift + lift*rearAxleDistance)/(frontAxleDistance + rearAxleDistance));
            Ry_diff = -1 * ((CGz*drag - CoPz*drag - CGx*lift + CoPx*lift + frontAxleDistance*lift)/(frontAxleDistance + rearAxleDistance));
            
            deltaFz = zeros(length(Velocity), 4);
            deltaFz(:, 1) = Fy_diff / 2;
            deltaFz(:, 2) = Fy_diff / 2;
            deltaFz(:, 3) = Ry_diff / 2;
            deltaFz(:, 4) = Ry_diff / 2;
        end
            
    end
    
end

