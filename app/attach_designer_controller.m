function controller=attach_designer_controller(main)
% ATTACH_DESIGNER_CONTROLLER Use three native App Designer layouts with shared logic.
root=fileparts(fileparts(mfilename('fullpath')));
% Assemble the initial editor layout before showing the application window.
main.UIFigure.Visible='off';
editor=[]; noise=[];
try
    editor=TrajectoryEditorApp; editor.UIFigure.Visible='off';
    noise=TrajectoryNoiseApp; noise.UIFigure.Visible='off';
    views=struct('Main',main,'Editor',editor,'Noise',noise);
    controller=TrajectoryAppController(root,'on',views);
catch err
    if ~isempty(editor) && isvalid(editor), delete(editor); end
    if ~isempty(noise) && isvalid(noise), delete(noise); end
    rethrow(err);
end
end
