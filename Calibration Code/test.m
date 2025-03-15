%% Initialization
clc;
clear;
close all;

%% Path Configuration
baseDir = 'C:/Users/alexl/Desktop/ARAE_Calibration/Human Testing/';
subjectFolders = dir(fullfile(baseDir, 'subject*')); % Get all subject folders

% Define muscle names based on the file format
muscleNames = ["R.Biceps Br.", "R.Lat. Triceps", "R.Ant.Deltoid", ...
               "R.Post. Deltoid", "R.Lat.Dorsi", "R.Pect. Major"];

% Define activities (rows) and muscles (columns)
activities = ["FR", "LR", "drink", "scoop"];
muscles = ["RBicepsBr", "RLatTriceps", "RAntDeltoid", ...
           "RPostDeltoid", "RLatDorsi", "RPectMajor"];

% Define collection modes (Norobot, Nominal, Personalised)
collectionModes = ["Norob", "Nominal", "Personalised"];
comparisonModes = ["Nominal", "Personalised"]; % Compared against "Norob"
collectionModes_labels = replace(collectionModes, "Personalised", "Personalized");
comparisonModes_labels = replace(comparisonModes, "Personalised", "Personalized");

newMuscleOrder = [1, 3, 6, 2, 4, 5];

% Sampling frequency (Hz)
fs = 2000;

% Define Butterworth Bandpass Filter (20–500 Hz)
[b_bandpass, a_bandpass] = butter(4, [20 500] / (fs / 2), 'bandpass');  

% Define Notch Filter at 50 Hz
notchFreq = 50;  
notchBW = 50 / fs;  % Small value relative to fs
[bn, an] = iirnotch(notchFreq / (fs / 2), notchBW);

% Number of subjects
numSubjects = length(subjectFolders);
numActivities = length(activities);
numMuscles = length(muscles);
numModes = length(collectionModes);

% Initialize matrices for storing MAV & MDF across subjects
MAV_subjects = nan(numActivities, numMuscles, numModes, numSubjects);
MDF_subjects = nan(numActivities, numMuscles, numModes, numSubjects);

%% Loop Through Subjects & Extract Features
for s = 1:numSubjects
    subjectID = subjectFolders(s).name;
    dataDir = fullfile(baseDir, subjectID, 'sEMG'); % sEMG folder path
    fileList = dir(fullfile(dataDir, '*.txt')); % Get all files in the sEMG folder
    
    % Initialize storage for this subject
    sEMG_features = struct();

    % Process each sEMG file
    for i = 1:length(fileList)
        filename = fullfile(dataDir, fileList(i).name);
        fileKey = fileList(i).name;
        
        % Detect import options
        opts = detectImportOptions(filename, 'Delimiter', '\t'); 
        opts.VariableNamingRule = 'preserve';
        opts.DataLines = [2 Inf]; 
        opts.VariableNamesLine = 1;

        % Read the data into a table
        dataTable = readtable(filename, opts);

        % Categorize file based on naming convention
        if contains(fileKey, 'Norob', 'IgnoreCase', true)
            category = 'Norob';
        elseif contains(fileKey, 'Nominal', 'IgnoreCase', true)
            category = 'Nominal';
        elseif contains(fileKey, 'Personalised', 'IgnoreCase', true)
            category = 'Personalised';
        else
            continue; % Skip unrecognized files
        end

        % Convert filename into valid struct field name
        subCategory = erase(fileKey, '.txt'); 
        subCategory = regexprep(subCategory, '[_\s\-.]', '');

        % Ensure category structure exists
        if ~isfield(sEMG_features, category)
            sEMG_features.(category) = struct();
        end
        if ~isfield(sEMG_features.(category), subCategory)
            sEMG_features.(category).(subCategory) = struct();
        end

        % Extract & filter each muscle signal
        for mi = 1:numMuscles
            m = newMuscleOrder(mi);
            muscleField = regexprep(muscleNames(m), '[_\s\-.]', '');
            muscleField = matlab.lang.makeValidName(muscleField);
            rawSignal = dataTable{:, m+1};

            % Handle NaN or Inf values
            if any(isnan(rawSignal)) || any(isinf(rawSignal))
                rawSignal = fillmissing(rawSignal, 'linear');
            end

            % Apply filters
            bandpassFiltered = filtfilt(b_bandpass, a_bandpass, rawSignal);
            notchFiltered = filtfilt(bn, an, bandpassFiltered);

            % Compute MAV & MDF
            MAV = mean(abs(notchFiltered));
            [Pxx, F] = pwelch(notchFiltered, [], [], [], fs);
            cumulativePower = cumsum(Pxx);
            totalPower = cumulativePower(end);
            MF_index = find(cumulativePower >= totalPower / 2, 1);
            MedianFreq = F(MF_index);

            % Store features
            sEMG_features.(category).(subCategory).(muscleField).MAV = MAV;
            sEMG_features.(category).(subCategory).(muscleField).MedianFreq = MedianFreq;

            % Store in subject-wise matrix
            actIndex = find(activities == erase(subCategory, category));
            modeIndex = find(collectionModes == category);
            MAV_subjects(actIndex, m, modeIndex, s) = MAV;
            MDF_subjects(actIndex, m, modeIndex, s) = MedianFreq;
        end
    end
end

%% Compute Mean MAV & MDF Across Subjects
MAV_avg = mean(MAV_subjects, 4, 'omitnan');
MDF_avg = mean(MDF_subjects, 4, 'omitnan');

%% Compute MAV & MDF Differences (Nominal/Personalised vs Norob)
MAV_diff_subjects = nan(numActivities, numMuscles, length(comparisonModes), numSubjects);
MDF_diff_subjects = nan(numActivities, numMuscles, length(comparisonModes), numSubjects);

for s = 1:numSubjects
    for c = 1:length(comparisonModes)
        mode1 = comparisonModes(c); % Nominal or Personalised
        mode2 = "Norob"; % Always comparing against Norob
        
        for a = 1:numActivities
            for mi = 1:numMuscles
                m = newMuscleOrder(mi);
                % Extract values for this subject
                MAV1 = MAV_subjects(a, m, collectionModes == mode1, s);
                MAV2 = MAV_subjects(a, m, collectionModes == mode2, s);
                MDF1 = MDF_subjects(a, m, collectionModes == mode1, s);
                MDF2 = MDF_subjects(a, m, collectionModes == mode2, s);

                % Compute Differences
                if ~isnan(MAV1) && ~isnan(MAV2)
                    MAV_diff_subjects(a, m, c, s) = MAV1 - MAV2;
                end
                if ~isnan(MDF1) && ~isnan(MDF2)
                    MDF_diff_subjects(a, m, c, s) = MDF1 - MDF2;
                end
            end
        end
    end
end

%% Compute Mean Differences Across Subjects
MAV_diff_avg = mean(MAV_diff_subjects, 4, 'omitnan');
MDF_diff_avg = mean(MDF_diff_subjects, 4, 'omitnan');

%% **Fix Row-Wise Scaling: Compute Maximum Values Per Row**
maxMAV_row = max(MAV_avg, [], [2, 3], 'omitnan'); % Max MAV for each row
maxMDF_row = max(MDF_avg, [], [2, 3], 'omitnan'); % Max MDF for each row
maxMAV_diff_row = max(abs(MAV_diff_avg), [], [2, 3], 'omitnan'); % Max MAV difference per row
maxMDF_diff_row = max(abs(MDF_diff_avg), [], [2, 3], 'omitnan'); % Max MDF difference per row

%% Prepare Data for Export in Table Format
MAV_csv_filename = fullfile(baseDir, 'MAV_Difference_Comparison.csv');
MDF_csv_filename = fullfile(baseDir, 'MDF_Difference_Comparison.csv');

% Create unique column headers
header1 = [{'Activity'}, strcat(repelem(muscles, length(comparisonModes)), '_', repmat(comparisonModes, 1, numMuscles))];

% Ensure headers are row cell arrays
header1 = reshape(header1, 1, []);

% Initialize data storage
MAV_data = cell(numActivities, numMuscles * length(comparisonModes) + 1);
MDF_data = cell(numActivities, numMuscles * length(comparisonModes) + 1);

% Populate data rows
for a = 1:numActivities
    MAV_data{a, 1} = activities(a);
    MDF_data{a, 1} = activities(a);
    col = 2; % Start from second column
    
    for mi = 1:numMuscles
        m = newMuscleOrder(mi);
        for c = 1:length(comparisonModes)
            MAV_data{a, col} = MAV_diff_avg(a, m, c);
            MDF_data{a, col} = MDF_diff_avg(a, m, c);
            col = col + 1;
        end
    end
end

% Convert to tables
MAV_table = cell2table(MAV_data, 'VariableNames', header1);
MDF_table = cell2table(MDF_data, 'VariableNames', header1);

%% Write CSV Files
writetable(MAV_table, MAV_csv_filename);
disp(['MAV difference data saved to: ', MAV_csv_filename]);

writetable(MDF_table, MDF_csv_filename);
disp(['MDF difference data saved to: ', MDF_csv_filename]);

%% **Plot MAV & MDF Comparisons Across Collection Modes (Row-Wise Scaling)**
figure('Name', 'Mean MAV Comparison', 'NumberTitle', 'off');
for a = 1:numActivities
    rowMaxMAV = maxMAV_row(a) * 1.1;
     for mi = 1:numMuscles
        m = newMuscleOrder(mi);
        subplot(numActivities, numMuscles, (a-1) * numMuscles + mi);
        data = squeeze(MAV_avg(a, m, :));
        bar(data, 'FaceColor', 'flat');
        set(gca, 'XTick', 1:numModes, 'XTickLabel', collectionModes_labels);
        ylabel('Mean MAV');
        title(sprintf('%s - %s', activities(a), muscles(m)), 'Interpreter', 'none');
        ylim([0, rowMaxMAV]);
        grid on;
    end
end
sgtitle('Mean MAV Comparison Across Collection Modes');

figure('Name', 'Mean MDF Comparison', 'NumberTitle', 'off');
for a = 1:numActivities
    rowMaxMDF = maxMDF_row(a) * 1.1;
    for mi = 1:numMuscles
        m = newMuscleOrder(mi);
        subplot(numActivities, numMuscles, (a-1) * numMuscles + mi);
        data = squeeze(MDF_avg(a, m, :));
        bar(data, 'FaceColor', 'flat');
        set(gca, 'XTick', 1:numModes, 'XTickLabel', collectionModes_labels);
        ylabel('Mean MDF (Hz)');
        title(sprintf('%s - %s', activities(a), muscles(m)), 'Interpreter', 'none');
        ylim([0, rowMaxMDF]);
        grid on;
    end
end
sgtitle('Mean MDF Comparison Across Collection Modes');

%% **Plot MAV & MDF Differences (Nominal/Personalised vs Norob)**
figure('Name', 'Mean MAV Change Comparison', 'NumberTitle', 'off');
for a = 1:numActivities
    rowMaxMAV_diff = maxMAV_diff_row(a) * 1.1;
    for mi = 1:numMuscles
        m = newMuscleOrder(mi);
        subplot(numActivities, numMuscles, (a-1) * numMuscles + mi);
        data = squeeze(MAV_diff_avg(a, m, :));
        
        % Determine background color based on the comparisons
        MAV_Nominal = data(1); % Norob vs Nominal
        MAV_Personalised = data(2); % Norob vs Personalised
        
        if MAV_Nominal < MAV_Personalised
            bgColor = [1, 0.8, 0.8]; % Light Red
        elseif MAV_Personalised < MAV_Nominal
            bgColor = [0.8, 0.8, 1]; % Light Blue
        else
            bgColor = [1, 1, 1]; % Default White
        end
        
        % Plot bar chart
        bar(data, 'FaceColor', 'flat');
        set(gca, 'XTick', 1:length(comparisonModes), 'XTickLabel', comparisonModes_labels);
        ylabel('Mean Δ MAV');
        title(sprintf('%s - %s', activities(a), muscles(m)), 'Interpreter', 'none');
        ylim([-rowMaxMAV_diff, rowMaxMAV_diff]);
        grid on;
        
        % Set subplot background color
        set(gca, 'Color', bgColor);
    end
end
sgtitle('Mean Change in MAV: Nominal/Personalized vs Norob');

figure('Name', 'Mean MDF Change Comparison', 'NumberTitle', 'off');
for a = 1:numActivities
    rowMaxMDF_diff = maxMDF_diff_row(a) * 1.1;
    for mi = 1:numMuscles
        m = newMuscleOrder(mi);
        subplot(numActivities, numMuscles, (a-1) * numMuscles + mi);
        data = squeeze(MDF_diff_avg(a, m, :));
        
        % Determine background color based on the comparisons
        MDF_Nominal = data(1); % Norob vs Nominal
        MDF_Personalised = data(2); % Norob vs Personalised
        
        if MDF_Nominal > MDF_Personalised
            bgColor = [1, 0.8, 0.8]; % Light Red
        elseif MDF_Personalised > MDF_Nominal
            bgColor = [0.8, 0.8, 1]; % Light Blue
        else
            bgColor = [1, 1, 1]; % Default White
        end
        
        % Plot bar chart
        bar(data, 'FaceColor', 'flat');
        set(gca, 'XTick', 1:length(comparisonModes), 'XTickLabel', comparisonModes_labels);
        ylabel('Mean Δ MDF (Hz)');
        title(sprintf('%s - %s', activities(a), muscles(m)), 'Interpreter', 'none');
        ylim([-rowMaxMDF_diff, rowMaxMDF_diff]);
        grid on;
        
        % Set subplot background color
        set(gca, 'Color', bgColor);
    end
end
sgtitle('Mean Change in Median Frequency (MDF): Nominal/Personalized vs Norob');