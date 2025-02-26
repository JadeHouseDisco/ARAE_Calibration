 %% Init
clc
clear
close all

%%
% Define the directory containing the sEMG files
dataDir = 'C:/Users/alexl/Desktop/ARAE_Calibration/Human Testing/subject1/sEMG';

% Get a list of all .txt files in the directory
fileList = dir(fullfile(dataDir, '*.txt'));

% Define muscle names based on the file format (must match file's column headers exactly)
muscleNames = ["R.Biceps Br.", "R.Lat. Triceps", "R.Ant.Deltoid", ...
               "R.Post. Deltoid", "R.Lat.Dorsi", "R.Pect. Major"];

% Sampling frequency (Hz)
fs = 2000;

% Define Butterworth Bandpass Filter (20–500 Hz)
[b_bandpass, a_bandpass] = butter(4, [20 500] / (fs / 2), 'bandpass');  

% Define Notch Filter at 50 Hz
notchFreq = 50;  
notchBW = 50 / fs;  % Small value relative to fs
[bn, an] = iirnotch(notchFreq / (fs / 2), notchBW);

% Initialize structures to store filtered data and features
sEMG_data = struct('Norob', struct(), 'Nominal', struct(), 'Personalised', struct());
sEMG_features = struct(); % Feature storage

% Loop through each file and read the data
for i = 1:length(fileList)
    % Get the full file path and filename
    filename = fullfile(dataDir, fileList(i).name);
    fileKey = fileList(i).name; % Original file name
    
    % Detect import options and preserve original variable names
    opts = detectImportOptions(filename, 'Delimiter', '\t'); 
    opts.VariableNamingRule = 'preserve'; % Preserve original column headers
    opts.DataLines = [2 Inf]; % Skip the first line (title)
    opts.VariableNamesLine = 1; % First row contains variable names

    % Read the data into a table
    dataTable = readtable(filename, opts);
    
    % Categorize based on filename
    if contains(fileKey, 'Norob', 'IgnoreCase', true)
        category = 'Norob';
    elseif contains(fileKey, 'Nominal', 'IgnoreCase', true)
        category = 'Nominal';
    elseif contains(fileKey, 'Personalised', 'IgnoreCase', true)
        category = 'Personalised';
    else
        category = 'Uncategorized'; % Catch any unexpected file names
    end

    % Convert filename into valid struct field name (Remove _ and other special chars)
    subCategory = erase(fileKey, '.txt'); % Remove .txt
    subCategory = regexprep(subCategory, '[_\s\-.]', ''); % Remove _, spaces, hyphens, and dots

    % Ensure field name validity and store data
    if ~isfield(sEMG_data.(category), subCategory)
        sEMG_data.(category).(subCategory) = struct();
    end
    if ~isfield(sEMG_features, category)
        sEMG_features.(category) = struct();
    end
    if ~isfield(sEMG_features.(category), subCategory)
        sEMG_features.(category).(subCategory) = struct();
    end

    % Store Time vector separately
    sEMG_data.(category).(subCategory).Time = dataTable{:,1};

    % Apply filtering and extract features for each muscle
    for m = 1:length(muscleNames)
        muscleField = regexprep(muscleNames(m), '[_\s\-.]', ''); % Remove special chars
        muscleField = matlab.lang.makeValidName(muscleField); % Ensure MATLAB-valid name
        rawSignal = dataTable{:, m+1};  % Original signal

        % Handle NaN or Inf values in the signal
        if any(isnan(rawSignal)) || any(isinf(rawSignal))
            rawSignal = fillmissing(rawSignal, 'linear'); % Interpolate missing values
        end

        % Apply Bandpass Filter (20–500 Hz)
        bandpassFiltered = filtfilt(b_bandpass, a_bandpass, rawSignal);

        % Apply Notch Filter at 50 Hz
        notchFiltered = filtfilt(bn, an, bandpassFiltered);

        % Store the filtered signal
        sEMG_data.(category).(subCategory).(muscleField) = notchFiltered;

        % **Feature Extraction**
        % Compute Mean Absolute Value (MAV)
        MAV = mean(abs(notchFiltered));

        % Compute Median Frequency (MF)
        [Pxx, F] = pwelch(notchFiltered, [], [], [], fs); % Compute Power Spectral Density
        cumulativePower = cumsum(Pxx); % Cumulative sum of power
        totalPower = cumulativePower(end); % Total power
        MF_index = find(cumulativePower >= totalPower / 2, 1); % Find frequency at 50% power
        MedianFreq = F(MF_index); % Extract median frequency

        % Store the extracted features
        sEMG_features.(category).(subCategory).(muscleField).MAV = MAV;
        sEMG_features.(category).(subCategory).(muscleField).MedianFreq = MedianFreq;
    end
end

%% Plot values
% Define testing categories (collection modes)
collectionModes = ["Norob", "Nominal", "Personalised"];

% Define activity names (row headers)
activities = ["drink", "FR", "LR"];

% Define muscle names (column headers)
muscles = ["RBicepsBr", "RLatTriceps", "RAntDeltoid", ...
           "RPostDeltoid", "RLatDorsi", "RPectMajor"];

% Number of rows (activities) and columns (muscles)
numActivities = length(activities);
numMuscles = length(muscles);

% Initialize matrices for MAV and MDF
MAV_matrix = nan(numActivities, numMuscles, length(collectionModes));
MDF_matrix = nan(numActivities, numMuscles, length(collectionModes));

% Extract MAV & MDF from sEMG_features
for c = 1:length(collectionModes)
    category = collectionModes(c);
    for a = 1:numActivities
        activity = strcat(category, activities(a)); % Generate field name (e.g., Norobdrink)
        if isfield(sEMG_features.(category), activity)
            for m = 1:numMuscles
                muscle = muscles(m);
                if isfield(sEMG_features.(category).(activity), muscle)
                    MAV_matrix(a, m, c) = sEMG_features.(category).(activity).(muscle).MAV;
                    MDF_matrix(a, m, c) = sEMG_features.(category).(activity).(muscle).MedianFreq;
                end
            end
        end
    end
end

% First Display: MAV Comparison
figure('Name', 'MAV Comparison', 'NumberTitle', 'off');
for a = 1:numActivities
    for m = 1:numMuscles
        subplot(numActivities, numMuscles, (a-1)*numMuscles + m);
        % Extract data and reshape to 1D
        data = squeeze(MAV_matrix(a, m, :));
        bar(data, 'FaceColor', 'flat');
        set(gca, 'XTick', 1:length(collectionModes), 'XTickLabel', collectionModes);
        ylabel('MAV');
        title(sprintf('%s - %s', activities(a), muscles(m)), 'Interpreter', 'none');
        ylim([0, max(MAV_matrix(:))*1.1]);
        grid on;
    end
end
sgtitle('MAV Comparison Across Collection Modes');

% Second Display: MDF Comparison
figure('Name', 'MDF Comparison', 'NumberTitle', 'off');
for a = 1:numActivities
    for m = 1:numMuscles
        subplot(numActivities, numMuscles, (a-1)*numMuscles + m);
        % Extract data and reshape to 1D
        data = squeeze(MDF_matrix(a, m, :));
        bar(data, 'FaceColor', 'flat');
        set(gca, 'XTick', 1:length(collectionModes), 'XTickLabel', collectionModes);
        ylabel('MDF (Hz)');
        title(sprintf('%s - %s', activities(a), muscles(m)), 'Interpreter', 'none');
        ylim([0, max(MDF_matrix(:))*1.1]);
        grid on;
    end
end
sgtitle('Median Frequency (MDF) Comparison Across Collection Modes');

%% Plot difference
%Define testing categories
collectionModes = ["Norob", "Nominal", "Personalised"];
comparisonModes = ["Nominal", "Personalised"]; % Only these will be compared against "Norob"

% Define activities (rows) and muscles (columns)
activities = ["drink", "FR", "LR"];
muscles = ["RBicepsBr", "RLatTriceps", "RAntDeltoid", ...
           "RPostDeltoid", "RLatDorsi", "RPectMajor"];

numActivities = length(activities);
numMuscles = length(muscles);

%bInitialize matrices for MAV and MDF comparison
MAV_comparison = nan(numActivities, numMuscles, length(comparisonModes));
MDF_comparison = nan(numActivities, numMuscles, length(comparisonModes));

% Extract MAV & MDF from sEMG_features
for c = 1:length(comparisonModes)
    mode1 = comparisonModes(c); % Nominal or Personalised
    mode2 = "Norob"; % Always comparing against Norobot
    
    for a = 1:numActivities
        act1 = strcat(mode1, activities(a)); % e.g., Nominaldrink
        act2 = strcat(mode2, activities(a)); % e.g., Norobdrink
        
        if isfield(sEMG_features.(mode1), act1) && isfield(sEMG_features.(mode2), act2)
            for m = 1:numMuscles
                muscle = muscles(m);
                if isfield(sEMG_features.(mode1).(act1), muscle) && ...
                   isfield(sEMG_features.(mode2).(act2), muscle)

                    MAV1 = sEMG_features.(mode1).(act1).(muscle).MAV;
                    MAV2 = sEMG_features.(mode2).(act2).(muscle).MAV;
                    MDF1 = sEMG_features.(mode1).(act1).(muscle).MedianFreq;
                    MDF2 = sEMG_features.(mode2).(act2).(muscle).MedianFreq;

                    MAV_comparison(a, m, c) = MAV1 - MAV2; % Difference (Change in MAV)
                    MDF_comparison(a, m, c) = MDF1 - MDF2; % Difference (Change in MDF)
                end
            end
        end
    end
end

% First Display: MAV Change (Nominal/Personalised vs Norobot)
figure('Name', 'MAV Change Comparison', 'NumberTitle', 'off');
for a = 1:numActivities
    for m = 1:numMuscles
        subplot(numActivities, numMuscles, (a-1)*numMuscles + m);
        data = squeeze(MAV_comparison(a, m, :));
        bar(data, 'FaceColor', 'flat');
        set(gca, 'XTick', 1:length(comparisonModes), 'XTickLabel', comparisonModes);
        ylabel('Δ MAV');
        title(sprintf('%s - %s', activities(a), muscles(m)), 'Interpreter', 'none');
        ylim([-max(abs(MAV_comparison(:)))*1.1, max(abs(MAV_comparison(:)))*1.1]); % Symmetric limits
        grid on;
    end
end
sgtitle('Change in MAV: Nominal/Personalised vs Norobot');

%bSecond Display: MDF Change (Nominal/Personalised vs Norobot)
figure('Name', 'MDF Change Comparison', 'NumberTitle', 'off');
for a = 1:numActivities
    for m = 1:numMuscles
        subplot(numActivities, numMuscles, (a-1)*numMuscles + m);
        data = squeeze(MDF_comparison(a, m, :));
        bar(data, 'FaceColor', 'flat');
        set(gca, 'XTick', 1:length(comparisonModes), 'XTickLabel', comparisonModes);
        ylabel('Δ MDF (Hz)');
        title(sprintf('%s - %s', activities(a), muscles(m)), 'Interpreter', 'none');
        ylim([-max(abs(MDF_comparison(:)))*1.1, max(abs(MDF_comparison(:)))*1.1]); % Symmetric limits
        grid on;
    end
end
sgtitle('Change in Median Frequency (MDF): Nominal/Personalised vs Norobot');