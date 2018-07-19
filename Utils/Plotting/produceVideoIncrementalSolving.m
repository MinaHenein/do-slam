function produceVideoIncrementalSolving(solver)

frames = [];
for i=1:size(solver,2)
    iSolver = solver(i);
    frames = [frames, iSolver.frames];
end

nDeleted = 0;
for j=1:size(frames,2)
    if size(frames(j-nDeleted).cdata,1)~= 450 || size(frames(j-nDeleted).cdata,2)~= 570
        frames(j-nDeleted) = [];
        nDeleted = nDeleted+1;
    end
end

implay(frames)

end