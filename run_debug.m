%% MATLAB: Safe wrapper for QHYCCD ring capture
% Set Python environment (adjust path to your Python3)
pyenv('Version', '/Users/ajeem/matlab_pyenv/bin/python3');

% Path to Python script
pythonScript = '/Users/ajeems/Downloads/LU/phase shift files/final ver/QHYCCD_ring_sync.py';

% Call Python script safely
try
    % Use MATLAB's Python interface
    py.runfile(pythonScript);
    disp('Python script executed successfully.');
catch ME
    error('Python script execution failed:\n%s', ME.message);
end

% Base folder where Python creates sample folders
baseFolder = '/Users/ajeems/Downloads/LU/phase shift files/final ver';

% Find the newest folder in the base folder
folders = dir(baseFolder);
folders = folders([folders.isdir] & ~ismember({folders.name},{'.','..'}));
[~, idx] = max([folders.datenum]); % latest folder
newestFolder = fullfile(baseFolder, folders(idx).name);

% Add folder with ring_alg.m to path
addpath('/Users/ajeems/Downloads/LU/phase shift files/final ver');

% Change directory to newest sample folder
cd(newestFolder);

% Run the MATLAB function
ring_alg;
