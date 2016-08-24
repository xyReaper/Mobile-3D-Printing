%version 1.0
function [flag]=are_they_equal(N1,N2)
if ((abs(N1 - N2)) <=10.0)
    flag = true;    
else
    flag = false;
end