function res = normalize_orientation(orientation, type)

if type == "euler"
    res = zeros(length(orientation), 3);
    %     
else
    res = zeros(length(orientation), 4);
end

rotm1 = quat2rotm(orientation(1,:));
    
for i = 1:length(orientation)
    rotm = rotm1' * quat2rotm(orientation(i, :));
    if type == "euler"
        res(i, :) = rotm2eul(rotm);
    else
        res(i, :) = rotm2quat(rotm);
    end
end

end