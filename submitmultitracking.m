% Load the video file & set up the video player
videoFReader = vision.VideoFileReader('racetrackcars.mov','VideoOutputDataType','double');
videoPlayer = vision.DeployableVideoPlayer;

% Initialise tracking structure and parameters
tracks = struct(...
    'id', {}, ...                      % Track ID
    'centroid', {}, ...                % Current centroid of the track
    'kalmanFilter', {}, ...            % Kalman filter for each track
    'consecutiveInvisibleCount', {}); % Invisible frame count
nextId = 1;                            % ID for the next new track

% Kalman filter configuration
param.motionModel = 'ConstantVelocity'; % Assumes constant velocity motion, explained in report in further depth
param.initialEstimateError = [1e6 1e6]; % Large initial uncertainty
param.motionNoise = [20 20];            % Process noise covariance
param.measurementNoise = 10;            % Measurement of noise covariance

% Tracking parameters
maxInvisibleCount = 10;                % Maximum allowed invisible frames for this algorithm
costOfNonAssignment = 50;              % Cost for unassigned tracks (weighting factor)

% Initialises frame counters as well as track history
frameCount = 0;
trackHistory = [];                     % Stores tracking history

% Loop through each frame in the video
while ~isDone(videoFReader)
    videoFrame = step(videoFReader);   % Read the current video frame
    frameCount = frameCount + 1;       % Increment frame count
    
    % Detects objects and their centroids in the frame
    [~, centroids] = segmentBall(videoFrame, 2000); % Minimum area for detection
    
    % Mark detected centroids on the frame, assigned as a black circle 
    if ~isempty(centroids)
        videoFrame = insertMarker(videoFrame, centroids, 'x', 'color', 'black', 'size', 5); 
    end
    
    nDetections = size(centroids, 1);  % Number of detected objects
    nTracks = length(tracks);          % Number of active tracks
    
    % Predict the next state of each track using the Kalman filter
    for idx = 1:nTracks
        tracks(idx).centroid = predict(tracks(idx).kalmanFilter);
    end
    
    % Compute assignment cost matrix for tracks and detections
    cost = zeros(nTracks, nDetections);
    for idx = 1:nTracks
        cost(idx, :) = distance(tracks(idx).kalmanFilter, centroids); % Compute Euclidean distance
    end
    
    % Assign detections to tracks
    [assignments, unassignedTracks, unassignedDetections] = ...
        assignDetectionsToTracks(cost, costOfNonAssignment);
    
    % Update assigned tracks with corresponding detections
    for idx = 1:size(assignments, 1)
        trackIdx = assignments(idx, 1);
        detectionIdx = assignments(idx, 2);
        correct(tracks(trackIdx).kalmanFilter, centroids(detectionIdx, :));
        tracks(trackIdx).consecutiveInvisibleCount = 0;
    end
    
    % Handle unassigned tracks and remove lost tracks
    toBeDeleted = false(nTracks, 1);
    for idx = 1:length(unassignedTracks)
        ind = unassignedTracks(idx);
        tracks(ind).consecutiveInvisibleCount = tracks(ind).consecutiveInvisibleCount + 1;
        if tracks(ind).consecutiveInvisibleCount > maxInvisibleCount
            toBeDeleted(ind) = true;
        end
    end
    tracks(toBeDeleted) = [];
    
    % Create new tracks for unassigned detections
    for idx = 1:size(unassignedDetections, 1)
        kalmanFilter = configureKalmanFilter(...
            param.motionModel, ...
            centroids(unassignedDetections(idx), :), ...
            param.initialEstimateError, ...
            param.motionNoise, ...
            param.measurementNoise);
        
        tracks(end+1).id = nextId;      % Assigns a new track ID
        tracks(end).kalmanFilter = kalmanFilter;
        tracks(end).centroid = centroids(unassignedDetections(idx), :);
        tracks(end).consecutiveInvisibleCount = 0;
        nextId = nextId + 1; 
    end
  %  This loop handles the creation of new tracks for detection, that were not assigned to existing tracks already. For each unassigned detection, it will configure a new Kalman filter. 
    % Stores the active track IDs as well as history
    activeTrackIDs = [tracks.id];
    trackHistory(frameCount).frame = frameCount;
    trackHistory(frameCount).ids = activeTrackIDs;

    % Annotates the track on the frame, track 1 and track 2 as visible on
    % the animation
    for idx = 1:length(tracks)
        videoFrame = insertText(videoFrame, ...
            tracks(idx).centroid, ...
            ['Track: ' num2str(tracks(idx).id)], ...
            'FontSize', 20, ...
            'BoxColor', 'yellow', ...
            'TextColor', 'black', ...
            'BoxOpacity', 0.6);
    end
    
    % Displays the video frame
    step(videoPlayer, videoFrame);
    pause(0.1) % Slight pause used to establish better visibility & clearly visible when executing the program, runs much slower than the orginal video/animation.
end

% clear cache memory
release(videoPlayer);
release(videoFReader);

% Plot the tracking history graph
figure;
hold on;
for idx = 1:length(trackHistory)
    frame = trackHistory(idx).frame;
    ids = trackHistory(idx).ids;
    for idIdx = 1:length(ids)
        plot(frame, ids(idIdx), 'bo', 'MarkerSize', 5, 'MarkerFaceColor', 'b');
    end
end
title('Tracking IDs vs Frames');
xlabel('Frame Number');
ylabel('Track ID');
grid on;
hold off;

% Function to segment objects in the frame based on color thresholds
function [boxLoc, centroidLoc] = segmentBall(videoFrame, minArea)
    persistent hBlob
    if isempty(hBlob)
        hBlob = vision.BlobAnalysis;
        hBlob.AreaOutputPort = false;
        hBlob.ExcludeBorderBlobs = true;
        hBlob.MinimumBlobArea = minArea;
    end
    
    % Converts the video frame to HSV color space for segmentation
    Ihsv = rgb2hsv(videoFrame);
    hue = Ihsv(:, :, 1);
    sat = Ihsv(:, :, 2);
    val = Ihsv(:, :, 3);

    % Detects blue and red objects based on HSV thresholds
    BW_blue = (hue >= 0.55 & hue <= 0.65) & (sat > 0.3) & (val > 0.2);
    BW_red1 = (hue >= 0.0 & hue <= 0.05) & (sat > 0.4) & (val > 0.2);
    BW_red2 = (hue >= 0.9 & hue <= 1.0) & (sat > 0.4) & (val > 0.2);
    BW_red = BW_red1 | BW_red2;

    BW = BW_blue | BW_red;            % Combines detected regions
    BW = imopen(BW, strel('disk', 5)); % Morphological opening in order remove noise

    [centroidLoc, boxLoc] = step(hBlob, BW); % Extracts object centroids and bounding boxes
    boxLoc = round(boxLoc);
end