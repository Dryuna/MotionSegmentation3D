function [result mask] = downsample3(data, num)

result = 0;

[dims points frames] = size(data);

mask = 1:num:points;

result = data(:,mask,:);

end

