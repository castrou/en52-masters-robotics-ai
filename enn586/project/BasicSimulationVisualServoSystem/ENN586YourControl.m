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

kf = YourVariables.kf;

[kf.e_est, kf.P_prev] = kf.measure(kf.e_pred, kf.P_pred, e.actual, kf.H, kf.R);
B = Ja(:,control.dof)*YourVariables.dt;

err = kf.e_est;
YourVariables.var1=YourVariables.var1+pinv(Ja(:,control.dof))*err;

if YourVariables.first==1
    Derror2=pinv(Ja(:,control.dof))*zeros(size(err));
    YourVariables.first=0;
else
     Derror2=pinv(Ja(:,control.dof))*err-YourVariables.error_old;
end

YourVariables.error_old=pinv(Ja(:,control.dof))*err;
uactual =-YourVariables.gainP*pinv(Ja(:,control.dof))*err-YourVariables.gainD*Derror2+YourVariables.gainI*YourVariables.var1; 
YourVariables.disturbance_est=YourVariables.gainI*YourVariables.var1;

kf.B_prev = B;
kf.u_prev = uactual;

[kf.e_pred kf.P_pred] = kf.predict(kf.A, kf.e_est, kf.B_prev, kf.u_prev, kf.P_prev, kf.Q);

YourVariables.kf = kf;
