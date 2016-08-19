%Version 1.0

%enter the name if the file you want to analyse

[center,angle] = detection('data18_transformed.ply');

disp ('The new center is', center);
disp ('The angle shift in radians is -', angle);