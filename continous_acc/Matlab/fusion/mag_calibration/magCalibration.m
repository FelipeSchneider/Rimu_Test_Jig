function [DD2,Ca,Cb] = magCalibration(DD)
%magCalibration calibrates the magnetometer data
%   DD2 calibrated data
%   Ca  matrix calibration factor
%   Cb  vector (offset) calibration factor
%   DD  magnetometer readings data without calibration

Coeficientes = EllipsoidFitting(DD);

[Ca, Cb] = Quadric2Calibration(Coeficientes);

DD2 = DD * Ca' + repmat(Cb', length(DD), 1);

end

