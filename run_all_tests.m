function summary = run_all_tests(includeSimulink,includeUI)
% RUN_ALL_TESTS Run relocation checks and both original test collections.
if nargin<1, includeSimulink=false; end
if nargin<2, includeUI=false; end
setup_project7;
test_project7_structure;
summary=run_tests(includeSimulink,includeUI);
run_project7_tests;
end
