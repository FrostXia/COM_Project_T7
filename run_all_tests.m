function summary = run_all_tests(includeSimulink,includeUI)
% RUN_ALL_TESTS Verify shared algorithms, the runtime and native views.
if nargin<1, includeSimulink=false; end
if nargin<2, includeUI=false; end
setup_paths;
test_project_structure;
test_nominal_turn;
test_control_noise;
test_desired_duration(includeSimulink);
if includeUI, test_designer_views(includeSimulink); end
summary=run_tests(includeSimulink,includeUI);
test_controllers;
end
