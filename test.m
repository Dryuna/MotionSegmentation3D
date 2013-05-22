clear;

% Robust 3D Motion Segmentation via Lossy Compression
% Joseph Pan <cs.wzpan@gmail.com>
% Extended from http://perception.csl.illinois.edu/coding/motion/#Software
% The above code have included four example motion sequences to test their motion
% segmentation algorithm:
% 1R2RC: a checkerboard sequence, 
% arm: an articulated motion sequence, 
% cars10: a traffic sequence, and 
% oc1R2RC: a checkerboard sequence with missing entries

%% add path
addpath 'helpers'; 
epsilon = logspace(-5,3,101);

% cycle color scheme for painting clusters
mycolor=[  
0 1 0
0 0 1  
1 0 0  
1 0 1  
0 1 1  
1 1 0  
0 .5 0  
0 .75 .75  
0 0 0
] ; 

%% Trying a clean sequence (no incomplete or corrupted data)
% Here is code that will give you results for one clean sequence (The 'arm'
% sequence with projection down to R^5).

[rawData, trueLabels] = load_sequence('arm');

% 0. Extend to 3D. You don't need this if your data is already 3D
x = rawData(1,:,:);
y = rawData(2,:,:);
z = sin(x); % add a dimension z and simply let z=sin(x)
mymask = rawData(3,:,:);
rawData = [x;y;z;mymask];

% 1. Calculate the size of data
[dims points frames]=size(rawData);

% 2. Normalize the data
data = reshape(rawData,dims,points*frames);
data = normalise3dpts(data);
rawData = reshape(data, dims,points,frames);

% 3. Clustering
processedData = process_sequence3d(rawData, true);
result = try_sequence('arm', processedData, epsilon);
computedLabels = find_best_segmentation(result, processedData, 5, epsilon);

% 4. Draw the original data
fig1=figure(1); hold on;
title('arm data, extended into 3D');
for i=1:points
    x = squeeze(rawData(1,i,:));
    y = squeeze(rawData(2,i,:));
    z = squeeze(rawData(3,i,:));
    line(x,y,z); 
    xlabel( 'x' );
    ylabel( 'y' );
    zlabel( 'z' );
    grid on;
    view( 37.5, 30 );
end;
hold off;
    
% 5. Draw the clustered data
fig2=figure(2); hold on;
title('clusterred result');
for i=1:points
    x = squeeze(rawData(1,i,:));
    y = squeeze(rawData(2,i,:));
    z = squeeze(rawData(3,i,:));
    
    h=line(x,y,z);     
    set(h,'Color',mycolor(mod(computedLabels(i),9)+1,:)); 
    
    xlabel( 'x' );
    ylabel( 'y' );
    zlabel( 'z' );
    grid on;
    view( 37.5, 30 );
end;
hold off;
    
%% Trying a sequence with incomplete data
% Here is code that will give you results for one incomplete sequence (The
% 'oc1R2RC' sequence with projection down to R^d, where d is the 'sparsity
% preserving' dimension).

[rawData, trueLabels, mask] = load_sequence('oc1R2RC');

% 0. Extend to 3D. You don't need this if your data is already 3D
x = rawData(1,:,:);
y = rawData(2,:,:);
z = sin(x); % add a dimension z and simply let z=sin(x)
mymask = rawData(3,:,:);
rawData = [x;y;z;mymask];

% 1. Calculate the size of data
[dims points frames]=size(rawData);

% 2. Normalize the data
data = reshape(rawData,dims,points*frames);
data = normalise3dpts(data);
rawData = reshape(data,dims,points,frames);

% 3. Clustering
processedData = process_sequence3d(rawData, 'sparse', 'incomplete', mask);
result = try_sequence('oc1R2RC', processedData, epsilon);
computedLabels = find_best_segmentation(result, processedData, 5, epsilon);
    
% 4. Draw the original data
fig3=figure(3); hold on;
title('oc1R2RC data, extended into 3D');
for i=1:points
    x = squeeze(rawData(1,i,:));
    y = squeeze(rawData(2,i,:));
    z = squeeze(rawData(3,i,:));
    m = squeeze(rawData(4,i,:));       
    match=find(m);
    line(x(match),y(match),z(match));     
    xlabel( 'x' );
    ylabel( 'y' );
    zlabel( 'z' );
    grid on;
    view( 37.5, 30 );
end;
hold off;
    
% 5. Draw the clustered data
fig4=figure(4); hold on;
title('clusterred result');
for i=1:points
    x = squeeze(rawData(1,i,:));
    y = squeeze(rawData(2,i,:));
    z = squeeze(rawData(3,i,:));
    m = squeeze(rawData(4,i,:));        
    match=find(m);
    h=line(x(match),y(match),z(match));     
    set(h,'Color',mycolor(mod(computedLabels(i),9)+1,:)); 
    xlabel( 'x' );
    ylabel( 'y' );
    zlabel( 'z' );
    grid on;
    view( 37.5, 30 );
end;
hold off;    
    
%% Trying a sequence with corrupted data
% Here is code that will give you results for one incomplete sequence 
% (The 'oc1R2RC' sequence with projection down to R^d, where d is 
% the 'sparsity preserving' dimension).

[rawData, trueLabels, mask] = load_sequence('oc1R2RC');

% 0. Extend to 3D. You don't need this if your data is already 3D
x = rawData(1,:,:);
y = rawData(2,:,:);
z = sin(x); % add a dimension z and simply let z=sin(x)
mymask = rawData(3,:,:);
rawData = [x;y;z;mymask];

% 1. Calculate the size of data
[dims points frames]=size(rawData);

% 2. Normalize the data
data = reshape(rawData,dims,points*frames);
data = normalise3dpts(data);
rawData = reshape(data,dims,points,frames);

% 3. Clustering
processedData = process_sequence3d(rawData, 'sparse', 'corrupted');
result = try_sequence('oc1R2RC', processedData, epsilon);
computedLabels = find_best_segmentation(result, processedData, 5, epsilon);
    
% 4. Draw the original data
fig5=figure(5); hold on;
title('oc1R2RC data, extended into 3D');
for i=1:points
    x = squeeze(rawData(1,i,:));
    y = squeeze(rawData(2,i,:));
    z = squeeze(rawData(3,i,:));
    m = squeeze(rawData(4,i,:));       
    match=find(m);
    line(x(match),y(match),z(match));     
    xlabel( 'x' );
    ylabel( 'y' );
    zlabel( 'z' );
    grid on;
    view( 37.5, 30 );
end;
hold off;
    
% 5. Draw the clusterred data
fig6=figure(6); hold on;
title('clusterred result');
for i=1:points
    x = squeeze(rawData(1,i,:));
    y = squeeze(rawData(2,i,:));
    z = squeeze(rawData(3,i,:));
    m = squeeze(rawData(4,i,:));        
    match=find(m);
    h=line(x(match),y(match),z(match));     
    set(h,'Color',mycolor(mod(computedLabels(i),9)+1,:)); 
    xlabel( 'x' );
    ylabel( 'y' );
    zlabel( 'z' );
    grid on;
    view( 37.5, 30 );
end;
hold off;    
