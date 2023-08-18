% ENN586YourControl as a script function.

% You may modified this file.
% This is the location of your control design, everything being assessed.

% You can use functions: ImageJacobian, wrapToPi, pinv
% You can compose and call your own functions.
% You can use variables (with their value at this call line in GetENN586ImageControl): 
%       Imaging, features, references, control,  noise, e
% You can NOT use any information in ENN586Error (assumed unknown). 
%      Except you may use information repeated in the 'noise' variable.

% the standard control you are aiming to better is. (from our CDC 2017 paper)
% uactual = -control.gain*pinv(Ja(:,control.dof))*e.actual;

%Can use "YourVariables" to store information needed in different loops
 



YourVariables.var1=YourVariables.var1+pinv(Ja(:,control.dof))*e.actual;

if YourVariables.first==1
    Derror2=pinv(Ja(:,control.dof))*zeros(size(e.actual));
    YourVariables.first=0;
else
     Derror2=pinv(Ja(:,control.dof))*e.actual-YourVariables.error_old;
end

YourVariables.error_old=pinv(Ja(:,control.dof))*e.actual;

uactual =-YourVariables.gainP*pinv(Ja(:,control.dof))*e.actual-YourVariables.gainD*Derror2+YourVariables.gainI*YourVariables.var1; 

YourVariables.disturbance_est=YourVariables.gainI*YourVariables.var1;
