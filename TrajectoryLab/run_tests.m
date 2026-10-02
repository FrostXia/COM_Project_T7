function summary=run_tests(includeSimulink,includeUI)
% RUN_TESTS Execute project tests and persist their actual outcome.
if nargin<1, includeSimulink=true; end
if nargin<2, includeUI=false; end
root=fileparts(mfilename('fullpath'));
addpath(root,fullfile(root,'src'),fullfile(root,'adapters'),fullfile(root,'tests'));
folder=fullfile(root,'results'); if ~isfolder(folder), mkdir(folder); end
diary(fullfile(folder,'test_output.txt'));
cleanup=onCleanup(@()diary('off'));
summary=test_p7(includeSimulink,includeUI);
save(fullfile(folder,'test_summary.mat'),'summary');
end
