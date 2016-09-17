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
        TabName
        Name = '';
        
        BrakingMode % 'Hydraulic', 'Regen', 'Combined'
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
            RollingR = CarObject.Weight*CarObject.Tire.RollingResistance; % Rolling Resistance force for car configuration
            
            MotorRPM = CarObject.Driveline.OutputCurve(:,3);
            MotorT   = CarObject.Driveline.OutputCurve(:,4);  %in-lb
            MotorE   = CarObject.Driveline.OutputCurve(:,5);
            AxleRPM  = CarObject.Driveline.OutputCurve(:,1);
            AxleT    = CarObject.Driveline.OutputCurve(:,2); % in-lb
            WheelF   = AxleT/CarObject.Tire.Radius; % lbf
            
            % Calculate Car Velocity for each Axle RPM value
            Velocity = CarObject.Tire.Radius*AxleRPM*pi/30; % in/s
            % Calculate corresponding drag for each veloicty
            [~,~,Drag]=CarObject.CalculateAeroEffects(Velocity);
            
            MaxDrivelineA   = (WheelF - Drag - RollingR)/(CarObject.Keq*CarObject.Weight/(12*32.174)); % in/s^2
            MaxDrivelineGs  = MaxDrivelineA/(12*32.174);
            
            % Compare driveline acceleration to possible acceleration from
            % tires and reduce accelerations to those values
            maxForwardGsTire = interp1(CarObject.Tire.ForwardAccelerationMap.velocities, CarObject.Tire.ForwardAccelerationMap.accelerations, Velocity, 'spline');
            ForwardGs = min(MaxDrivelineGs, maxForwardGsTire);
            
            % Recalculate Driveline torque values based on available
            % traction.
            AdjustedWheelF = ForwardGs*(12/32.174);
            AdjustedAxleT = AdjustedWheelF * CarObject.Tire.Radius;
            AdjustedMotorT = AdjustedAxleT ./ AxleT .* MotorT;
            
            
            % Calculate power consumption for each motor rpm
            
            switch CarObject.TabName
                case 'Electric'
                    Power = (AdjustedMotorT.*MotorRPM./MotorE)*pi/30;
                case 'Combustion'
                    Power = (AdjustedMotorT./MotorT).*MotorE;
            end
            
            LateralGs = zeros(length(Velocity),1);
            
            % Tractive limit is reached at all of the indexes that were
            % previously adjusted to match tire acceleration
            [~, TractiveLimit] = ismember(ForwardGs, maxForwardGsTire);
            
            %    1       2     3        4            5         6           7     8         9        10
            LookUpTable = [Velocity,Drag,AxleRPM,MotorRPM,AdjustedMotorT,MotorE,Power,ForwardGs,LateralGs,TractiveLimit];
        end
        
        function [ LookUpTable ] = StraightDecTableGenerator(CarObject,Velocity,Drag)
            RollingR = CarObject.Weight*CarObject.Tire.RollingResistance; % Rolling Resistance force for car configuration
            
            
            if strcmp(CarObject.BrakingMode, 'Hydraulic')
                % Assume brakes use tire at full potential
                ForwardGs = -1 * interp1(CarObject.Tire.BrakingAccelerationMap.velocities, CarObject.Tire.BrakingAccelerationMap.accelerations, Velocity, 'spline');
            elseif strcmp(CarObject.BrakingMode, 'Regen')
                % Assume motor uses tires at full potential
                ForwardGs = -1 * interp1(CarObject.Tire.RegenAccelerationMap.velocities, CarObject.Tire.RegenAccelerationMap.accelerations, Velocity, 'spline');
            end
            
            % Calculate wheel force based on tire capability.
            WheelF = CarObject.Keq*CarObject.Weight*ForwardGs - Drag - RollingR;
            
            % Calculate axle and motor rpms based on velocity
            AxleRPM = Velocity/(CarObject.Tire.Radius*pi/30);
            MotorRPM = CarObject.Driveline.OutputCurve(round(AxleRPM)+1, 3);
            
            % Calculate applied brake/motor torque based on wheel force
            if strcmp(CarObject.BrakingMode, 'Regen')
                MotorTorque = -WheelF.*AxleRPM./MotorRPM*CarObject.Tire.Radius;
                BrakeTorque = zeros(length(MotorTorque),1);
            elseif strcmp(CarObject.BrakingMode, 'Hydraulic')
                BrakeTorque = WheelF*CarObject.Tire.Radius;
                MotorTorque = zeros(length(BrakeTorque),1);
            end
            
            if strcmp(CarObject.BrakingMode, 'Hydraulic')
                % Assume brakes use tire at full potential
                MotorPower = zeros(length(MotorTorque),1);
            elseif strcmp(CarObject.BrakingMode, 'Regen')
                % Assume motor uses tires at full potential
                Efficiency = .9 * .95 * .9; % Motor * Driveline * Battery
                MotorPower = (MotorTorque .* MotorRPM)*Efficiency * pi/30;
            end
            
            % Straight brake curve, therefore lateral Gs is always zero
            LateralGs = zeros(length(Velocity),1);
            
            % Tractive limit is reached at all points with ideal braking
            TractiveLimit = ones(length(ForwardGs),1);
            
            %    1       2     3        4       5           6        7           8               9
            LookUpTable = [Velocity,Drag,AxleRPM,MotorRPM,BrakeTorque,ForwardGs,LateralGs,TractiveLimit, MotorTorque, MotorPower];
        end
        
        function [ LookUpTable ] = CornerAccTableGenerator( CarObject,R,Velocity,Drag)
            
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
            % Find max forward Gs available from tires for each lateral G
            ForwardGs = CarObject.Tire.GGCurve(LateralGs,'Throttle', Velocity);
            
            % Calculate wheel forces, axle torque, and motor torque based
            % on given forward Gs
            WheelF = ForwardGs*CarObject.Weight*CarObject.Keq + Drag + RollingR;
            AxleT  = WheelF*CarObject.Tire.Radius;
            AxleRPM = Velocity*30/(pi*CarObject.Tire.Radius);
            
            % Round axle rpms to use as index for driveline output curve.
            AxleRPMIndexes = round(AxleRPM) + 1;
            
            % Find drive ratio at all AxleRPMs
            DriveRatio = CarObject.Driveline.OutputCurve(AxleRPMIndexes,3) ./ AxleRPM;
            
            % Pull available axle torques for axle rpms
            AxleTAvailable = CarObject.Driveline.OutputCurve(AxleRPMIndexes,2);
            
            % Find calculated axle torques that are more than the motor is
            % capable of
            I = AxleT > AxleTAvailable;
            % And set those values to available motor torque
            AxleT(I) = AxleTAvailable(I);
            
            % Tractive limit is reached at all indexes not limited by motor
            % torque
            TractiveLimit = ~I;
            
            % Recalculate motor torque, motor rpm, wheel force, and forward
            % Gs based on adjusted available torque values.
            
            MotorT = AxleT ./ DriveRatio;
            MotorRPM = AxleRPM .* DriveRatio;
            MotorE = CarObject.Driveline.OutputCurve(AxleRPMIndexes, 5);
            
            WheelF = AxleT/CarObject.Tire.Radius; % lbf
            ForwardGs = (WheelF - Drag - RollingR)/(CarObject.Keq*CarObject.Weight);
            
            % calculate power consumption based on motor torque, rpm and
            % efficiency
            %             Power = ((MotorT.*MotorRPM./MotorE)*pi/30);
            switch CarObject.TabName
                case 'Electric'
                    Power = ((MotorT.*MotorRPM./MotorE)*pi/30);
                case 'Combustion'
                    Power = MotorE;
            end
            %                  1      2     3       4        5      6     7       8         9          10
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
            BackGs = CarObject.Tire.GGCurve(LateralGs,'Brake',Velocity, CarObject.BrakingMode);
            
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
            MotorRPM = CarObject.Driveline.OutputCurve(round(AxleRPM) + 1, 3);
            
            % Tractive limit is reached at all indexes where braking torque
            % is less than the available braking torque
            TractiveLimit = BrakeTorque < sum(CarObject.Brakes.Torque);
            
            %                  1      2     3        4         5        6        7           8
            LookUpTable = [Velocity,Drag,AxleRPM,MotorRPM,BrakeTorque,BackGs,LateralGs,TractiveLimit, zeros(length(LateralGs),1), zeros(length(LateralGs),1)];
            
        end
        
        function [deltaFz,lift,drag] = CalculateAeroEffects(CarObject, Velocity)
            Velocity = Velocity / 12; % Convert in/s to ft/s;
            %   Load Distribution Equations from: Solve[{Fz + Rz + Lift == 0, -Fz*frontAxleDistance + Rz*rearAxleDistance - drag*(CoPz - CGz) + lift*(CoPx - CGx) == 0}, {Fz, Rz}]
            %   Velocity-ft/s rho-slug/ft^3 front_area-in^2
            q=.5.*CarObject.Rho.*Velocity.^2;   %Dynamic Pressure psf
            S=CarObject.FrontCrossSection/144;  %Convert to ft^2
            lift=CarObject.LiftCoefficient.*q.*S;   %lbf
            drag=CarObject.DragCoefficient.*q.*S;   %lbf

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

