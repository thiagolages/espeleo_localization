function [H] = quaternionToHomogeneous(quat)
    %quaternionToHomogeneous 
    %   Converts a Nx7 matrix containing (x, y, z, x, y, z, w) to 
    % a 4x4xN homogeneous transformation matrix
    N = size(quat,1);
    H = zeros(4,4,N);
    
    for i=1:N
        x = quat(i,4); y = quat(i,5); z = quat(i,6); w = quat(i,7);
        q = Quaternion([w,x,y,z]);                      % input is [w,x,y,z]
        R = q.R;                                        % rotation
        t = [quat(i,1), quat(i,2), quat(i,3)]';         % translation
        H(:,:,i) = rt2tr(R, t);
    end
    
end

