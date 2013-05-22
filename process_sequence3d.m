function [processedData isGrossError] = process_sequence3d(rawData, projection, mode, mask, log)

% process_sequence
%
%       process tracked feature correspondences. the raw data is assumed to
%       be in a 2xFxP matrix, where F is the number of frames and P is the
%       number of points. For each tracked feature, it coordinates in all
%       frames are stacked together to obtain a 2Fx1 vector. additionally, 
%       projection onto a low-dimensional subspace, or dealing with incomplete or
%       corrupted trajectories may also be required.
%
% Inputs:
%   rawData             - A matrix of the raw tracked feature
%                           correspondences.
%
%   projection          - if true, project the feature vector onto a
%                           5-dimensional space using PCA. if false, omit
%                           any projection. If equal to the string
%                           "sparse", project the feature vectors ont a
%                           d-dimension space using PCA, where d is the
%                           smallest integer that satisfies d >
%                           2*k*lg(N/d).
%
%   mode                - if equal to 'incomplete', then fill in incomplete trajectories 
%    (optional)           specified by mask. if equal to 'corrupted', detected and repair 
%                         corrupted trajectories. otherwise, the mode should be missing 
%                         or equal to 'clean'. 
%   mask                - if mode equals 'incomplete', then this is a required PxF matrix 
%    (optional)           whose ijth entry is 1 if the ith point is visible in the jth frame, 
%                         and 0 otherwise. this matrix is ignored for all other modes.        
%   log
%    (optional)         - the location of log
%                  
% Outputs:
%   processedData        - the processed motion data.
%
%   isGrossError         - if mode equals 'corrupted', then this is a 2FxP
%                          matrix whose ij-th entry is true if the ith
%                          entry of the jth trajectory was corrupted by a
%                          gross error.
% Dependencies:
%   repair_incomplete_data, repair_corrupted_data
%
% Mar. '08  Shankar Rao -- srrao@uiuc.edu

% Copyright 2008, University of Illinois. All rights reserved.


VERBOSE = true;
HASLOG = false;

[ignore sampleCount frameCount] = size(rawData); 

data = zeros(3*frameCount, sampleCount); % Joseph 2->3
if nargin < 3
    mode = 'clean';
end;
if nargin > 4
    HASLOG = true;
end;

if strcmp(mode, 'incomplete')
    isComplete = all(mask,2); % Joseph 2->3
    completeSampleCount = sum(isComplete);
    incompleteSampleCount = sum(~isComplete);

    completeData = zeros(3*frameCount, completeSampleCount);    % 2->3
    incompleteData = zeros(3*frameCount, incompleteSampleCount);% 2->3
    incompleteMask = ones(3*frameCount, incompleteSampleCount); % 2->3
    cSampleIndex = 1; 
    icSampleIndex = 1;
    for sampleIndex=1:sampleCount
        if isComplete(sampleIndex)
            completeData(:, cSampleIndex) = reshape(rawData(1:3, sampleIndex, :), 3*frameCount,1); % 2->3
            cSampleIndex = cSampleIndex + 1;
        else
            incompleteData(:, icSampleIndex) = reshape(rawData(1:3, sampleIndex, :), 3*frameCount,1); %2->3
            incompleteMask(1:3:3*frameCount-2, icSampleIndex) = mask(sampleIndex, :)'; % 1:2:2*frameCount-1 -> 1:3:3*frameCount-2
            incompleteMask(2:3:3*frameCount-1, icSampleIndex) = mask(sampleIndex, :)';   % 2:2:2*frameCount -> 2:3:3*frameCount-1
            incompleteMask(3:3:3*frameCount, icSampleIndex) = mask(sampleIndex, :)';   % add this line
            icSampleIndex = icSampleIndex + 1;
            if VERBOSE,
              message=sprintf('Completed sample %d of %d with %d missing points\n', icSampleIndex, incompleteSampleCount, sum(~mask(sampleIndex,:)));
            end;
            if HASLOG
              disp(message);
              fid=fopen(log,'at+');
              fprintf(fid,message);
              fclose(fid);
            end;            
        end;

    end;
    if HASLOG
            fid=fopen(log,'at+');
            fprintf(fid,'======================================================\n\n');
            fprintf(fid,'repairing data...\n');
            fclose(fid)
    end;
    repairedData = repair_incomplete_data(completeData, incompleteData, ~incompleteMask);
    data(:, isComplete) = completeData;
    data(:, ~isComplete) = repairedData;
elseif strcmp(mode, 'clean') || strcmp(mode, 'corrupted')
    for sampleIndex=1:sampleCount
        data(:, sampleIndex) = reshape(rawData(1:3, sampleIndex, :), 3*frameCount,1);   % 1:2 -> 1:3; 2*frameCount->3*frameCount
    end;
else
    error('Unknown mode for processing motion sequence data.');
end;

if strcmp(mode, 'corrupted')
    corruptedData = data;
    isGrossError = false(size(data));
    for sampleIndex = 1:sampleCount
        [data(:, sampleIndex), ignore, isGrossError(:, sampleIndex)] = ...
            repair_corrupted_data(corruptedData(:, [1:sampleIndex-1 sampleIndex+1:sampleCount]), corruptedData(:, sampleIndex));
        if VERBOSE,
            disp(sprintf('Repaired sample %d of %d, %d errors found', sampleIndex, sampleCount, sum(isGrossError(:, sampleIndex))));
        end;
        if HASLOG    
            message = sprintf('Repaired sample %d of %d, %d errors found\n', sampleIndex, sampleCount, sum(isGrossError(:, sampleIndex)));
            fid=fopen(log,'at+');
            fprintf(fid,message);
            fclose(fid);
        end;
    end;
end;

% projecting data into lower dimensional space
if HASLOG    
    message = sprintf('projecting data into lower dimensional space\n');
    fid=fopen(log,'at+');
    fprintf(fid,'======================================================\n\n');
    fprintf(fid,message);
    fclose(fid);        
end;
if projection,
    if strcmp(projection, 'sparse')
        dimensionCount = 1;
        while (dimensionCount < 8*log2(3*frameCount/dimensionCount))    % 2*frameCount -> 3*frameCount
            dimensionCount = dimensionCount + 1;
        end;
        if VERBOSE
            disp(sprintf('Calculated dimension = %d', dimensionCount));
        end;
        if HASLOG
            message = sprintf('Calculated dimension = %d \n', dimensionCount);
            fid=fopen(log,'at+');
            fprintf(fid,message);
            fclose(fid);        
        end;
    else
        dimensionCount = 5;
        if HASLOG
            message = sprintf('Dimension = %d\n', dimensionCount);
            fid=fopen(log,'at+');
            fprintf(fid,message);
            fclose(fid);        
        end;
    end;
    [U S] = svd(data, 0);
    basis = U(:, 1:dimensionCount);
    processedData = basis'*data;
else;
    dimensionCount = size(data, 1);
    processedData = data;
end