function outputFile = reproduce_run(snapshotName, outputDirectory)
% Re-run a published historical source snapshot without auto-starting animation.
% Example: reproduce_run('V12_35_3D.m', fullfile(tempdir,'vehicle_v12'))
% The original snapshot and current Live Scripts are never rewritten.
arguments
    snapshotName (1,:) char
    outputDirectory (1,:) char = fullfile(tempdir,'vehicle_project_results')
end
toolDir=fileparts(mfilename('fullpath'));
sourceDir=fullfile(toolDir,'..','Results','Run_Sources');
[folder,name,ext]=fileparts(snapshotName);
assert(isempty(folder)&&strcmp(ext,'.m'),'Choose a filename from Results/Run_Sources.');
sourcePath=fullfile(sourceDir,[name ext]); assert(isfile(sourcePath),'Unknown snapshot.');
if ~isfolder(outputDirectory),mkdir(outputDirectory);end
[ok,attributes]=fileattrib(outputDirectory);assert(ok);outputDirectory=attributes.Name;
temporaryDir=tempname;mkdir(temporaryDir);
previousDir=pwd;previousVisibility=get(0,'DefaultFigureVisible');
restore=onCleanup(@()restoreEnvironment(previousDir,previousVisibility)); %#ok<NASGU>
set(0,'DefaultFigureVisible','off');
code=fileread(sourcePath);
code=regexprep(code,'ensayo\.animar\s*=\s*true\s*;','ensayo.animar=false;');
code=strrep(code,'op3D.reproducir=true;','op3D.reproducir=false;');
temporaryScript=fullfile(temporaryDir,'run_snapshot.m');
fid=fopen(temporaryScript,'w','n','UTF-8');assert(fid>=0);
fprintf(fid,'%s',code);fclose(fid);
cd(temporaryDir);
outputFile=fullfile(outputDirectory,[name '_results.mat']);
executeSnapshot(temporaryScript,outputFile);
fprintf('Saved full-resolution results to %s\n',outputFile);
% The temporary script is retained for audit; no recursive deletion is used.
end

function executeSnapshot(repScriptPath,repOutputPath)
run(repScriptPath);
repNames={'resultados','resultadosBase','perfiles','perfilesBase','hist35', ...
    'pista','p','plan','planBase','ensayo','resumen','comparacion','conductor'};
repPayload=struct();
for repIndex=1:numel(repNames)
    repName=repNames{repIndex};
    if exist(repName,'var'),repPayload.(repName)=eval(repName);end
end
save(repOutputPath,'-struct','repPayload','-v7');
% Stop only the figures created by this snapshot, including its animation timer.
if exist('fig3D','var')&&isgraphics(fig3D),close(fig3D);end
end

function restoreEnvironment(previousDir,previousVisibility)
cd(previousDir);set(0,'DefaultFigureVisible',previousVisibility);
end
