function [WP,Odom] = loadFile(filename)

fid = fopen(filename)
lines = textscan(fid,'%s','Delimiter','\n\r')
lines = lines{1}

j = 1;
k = 1;

for i = 1:length(lines)
    temp = textscan(lines{i},'%s %f %f %f','Delimiter',',');
    
    header = temp{1}
    if strcmp(header{1},'ODOM')
    nums = temp(2:4);
    Odom.X(j) = nums{1}*1000;
    Odom.Y(j) = nums{2}*1000;
    Odom.Z(j) = nums{3}*1000;
        j = j+1;
    else
    nums = temp(2:4);
    WP.X(k) = nums{1}*1000;
    WP.Y(k) = nums{2}*1000;
    WP.Z(k) = nums{3}*1000;
    k = k+1;
    end
end

