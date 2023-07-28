function [ Error ] = GetENN586Parameters(range_error_status, range_error_type, range_error_size,feature_status,feature_offest)

% Author:       Jason Ford
% Date:         May 2023
% Details:      Sets parameters for ENN586 Assessment task
%               

% Further Info: Modified visual servoing code (see reference)
% Reference:    A. McFadyen, J. Ford and P. Corke
%               "Stable image-based visual servoing with unknown point feature correspondence"
%               2017 IEEE 56th Annual Conference on Decision and Control (CDC)
% Contact:      aaron.mcfadyen@qut.edu.au

%**************************************************************************
% Set Range Error Type
%**************************************************************************
%range_error=[range_error_status, range_error_type, range_error_size]
%feature_bias=[feature_error_status, feature_offset];

Error.range_status      =range_error_status;
Error.range_type        =range_error_type;
Error.range_size        =range_error_size;

Error.feature_status      =feature_status;
Error.feature_offset       =feature_offest;



