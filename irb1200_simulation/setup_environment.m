%% ─────────────────────────────────────────────────────────────
% File        : setup_environment.m
% Project     : Open-Source MATLAB Simulation — ABB IRB 1200
% Author      : Fernando Aquilino Gatell Valor
% Institution : Universidad Francisco de Vitoria — PFG 2024/25
% MATLAB      : R2025b | Robotics System Toolbox
% Description : Adds all subfolders to MATLAB path, checks required
%               toolboxes and verifies the URDF file exists, then
%               creates output directories.
% Inputs      : none
% Outputs     : none (side-effects: modified MATLAB path, created dirs)
% Dependencies: Robotics System Toolbox (license check)
%% ─────────────────────────────────────────────────────────────

%% 1 — Determine this script's directory ────────────────────────
script_dir = fileparts(mfilename('fullpath'));

%% 2 — Add all subfolders to the MATLAB path ────────────────────
% This makes robot/, environment/, ssm/, planners/, etc. discoverable.
addpath(genpath(script_dir));

%% 3 — Toolbox availability checks ─────────────────────────────
if ~license('test', 'Robotics_System_Toolbox')
    error(['setup_environment: Robotics System Toolbox is required ' ...
           'but not available on this licence. ' ...
           'Install it from Add-Ons or contact your IT admin.']);
end

if ~license('test', 'Statistics_Toolbox')
    warning(['setup_environment: Statistics and Machine Learning Toolbox ' ...
             'not found. Benchmark statistics (mean/std) will still run ' ...
             'using built-in MATLAB functions, but some advanced stats ' ...
             'may not be available.']);
end

%% 4 — Verify URDF file exists ──────────────────────────────────
% The ROS-Industrial ABB package must be present in the parent folder
% as abb-noetic-devel/ (the folder name used by this repo clone).
urdf_path = fullfile(script_dir, '..', 'abb-noetic-devel', ...
    'abb_irb1200_support', 'urdf', 'irb1200.urdf');
urdf_path = char(java.io.File(urdf_path).getCanonicalPath());

if ~isfile(urdf_path)
    error(['setup_environment: URDF not found at:\n  %s\n' ...
           'Ensure abb-noetic-devel/ is present in the parent folder:\n' ...
           '  %s\n' ...
           'Clone it from: https://github.com/ros-industrial/abb'], ...
          urdf_path, fullfile(script_dir, '..'));
end
fprintf('[setup_environment] URDF found: %s\n', urdf_path);

%% 5 — Create output directories ────────────────────────────────
results_dir  = fullfile(script_dir, 'results');
figures_dir  = fullfile(results_dir, 'figures');

dirs_to_create = {results_dir, figures_dir};
for k = 1:numel(dirs_to_create)
    d = dirs_to_create{k};
    if ~isfolder(d)
        mkdir(d);
        fprintf('[setup_environment] Created directory: %s\n', d);
    end
end

%% 6 — Print banner ─────────────────────────────────────────────
fprintf('\n');
fprintf('╔══════════════════════════════════════════════════╗\n');
fprintf('║  IRB 1200 Open-Source Simulation Environment    ║\n');
fprintf('║  Author : Fernando Aquilino Gatell Valor        ║\n');
fprintf('║  UFV — PFG 2024/25 — MATLAB R2025b              ║\n');
fprintf('╚══════════════════════════════════════════════════╝\n');
fprintf('\n');
