function TMatrix = body2ned(PHI,THETA, PSI)

% Generate transformation matrix to convert translational rates in body 
% frame to inertial/fixed frame (NED) based on Euler angles. 

Cpsi = cos(PSI);
Ctheta = cos(THETA);
Cphi =  cos(PHI);

Spsi = sin(PSI);
Stheta = sin(THETA);
Sphi = sin(PHI);

TMatrix(1,1) = Ctheta*Cpsi;
TMatrix(1,2) =  Sphi*Stheta*Cpsi - Cphi*Spsi;
TMatrix(1,3) =  Cphi*Stheta*Cpsi + Sphi*Spsi;

TMatrix(2,1) = Ctheta*Spsi;
TMatrix(2,2) = Sphi*Stheta*Spsi + Cphi*Cpsi;
TMatrix(2,3) = Cphi*Stheta*Spsi - Sphi*Cpsi;

TMatrix(3,1) = -Stheta;
TMatrix(3,2) = Sphi*Ctheta;
TMatrix(3,3) = Cphi*Ctheta;

end 