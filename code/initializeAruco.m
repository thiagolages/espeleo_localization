function [arucos] = initializeAruco()
    
    arucos = zeros(12,7); % array with [X Y Z X Y Z W] (pose w/ quaternions)
    
    for i=1:size(arucos,1)
        % position
        arucos(i,1) = i*2 + 1.0;        %X
        if mod(i,2) == 1 % if odd
            arucos(i,2) = 2.16 - 0.126; %Y if odd
        else
            arucos(i,2) = 0.126;         %Y if even
        end
        arucos(i,3) = 0.115;             %Z
        
        q = Quaternion(troty(-pi/2)*trotz(pi/2));
        q = q.double;
        % orientation in quaternions
        arucos(i,4) = q(2); %X = -0.5
        arucos(i,5) = q(3); %Y = -0.5
        arucos(i,6) = q(4); %Z = 0.5
        arucos(i,7) = q(1); %W = 0.5
        
    end
end

