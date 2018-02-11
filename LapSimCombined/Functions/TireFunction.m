function [friction_instant] = TireFunction(weight_on_tire,tire_choice)

%Tire Selection.  The equations that follow are based on tabulated Excel
%data.  The data was compiled based on a slip angle varrying between -5
%degrees and 13 degrees, an average between the values was taken at various
%normal loads.  With the average and corresponding normal load, a graph was
%created and the equation extrapolated to be used in the following MatLab
%code.

%Correction factor is 0.67 which is multiplied by the instantaneous
%coefficient of friction to obtain a real world coefficient of friction.
%Used 0.55 instead of 0.67 to get more realistic acceleration results as
%0.67 yielded longitudinal acceleration of .8g's and 0.55 comes closer to
%0.5g's.

%Tire 1=Goodyear 13"
%Tire 2=Hoosier 13"
%Tire 3=Hoosier Small 13"
%Tire 4=Michelin 13"

coefficient_of_friction_correction = 0.55;




if tire_choice==1
    friction_instant = coefficient_of_friction_correction*(7.2542E-11*weight_on_tire^4 - 6.1031E-08*weight_on_tire^3 + 1.5083E-05*weight_on_tire^2 - 1.8194E-03*weight_on_tire + 2.5472E+00);
elseif tire_choice==2
    friction_instant = coefficient_of_friction_correction*(9.1129E-11*weight_on_tire^4 - 7.0271E-08*weight_on_tire^3 + 1.6161E-05*weight_on_tire^2 - 1.7813E-03*weight_on_tire + 2.6989E+00);
elseif tire_choice==3
    friction_instant = coefficient_of_friction_correction*(1.0498E-10*weight_on_tire^4 - 1.0451E-07*weight_on_tire^3 + 3.4614E-05*weight_on_tire^2 - 4.5785E-03*weight_on_tire + 2.7045E+00);
elseif tire_choice==4
    friction_instant = coefficient_of_friction_correction*(-5.0728E-11*weight_on_tire^4 + 4.5286E-08*weight_on_tire^3 - 1.4487E-05*weight_on_tire^2 - 9.8030E-05*weight_on_tire + 2.1029E+00);
end




end

