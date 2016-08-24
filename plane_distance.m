function [modelheight]=plane_distance(plane1,plane2)

numer = abs(plane1.Parameters(3) + plane1.Parameters(4));
denom = sqrt(plane2.Parameters(1)^2 + plane2.Parameters(2)^2 + plane2.Parameters(3)^2);

modelheight= numer/denom;
end