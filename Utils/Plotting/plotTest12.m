sequence  = 6:19;
SE3Vertex = 'SE3 Motion Vertex initialized as identity';
noise = '[0.01;0.01;0.01]';

edges = [];
for i = sequence
    fid = fopen(strcat('Data/GraphFiles/test12_',...
        num2str(fibonacci(i)),'_1_1measurements.graph'),'r');
    n = 0;
    tline = fgetl(fid);
    while ischar(tline)
        tline = fgetl(fid);
        n = n+1;
    end
    edges = [edges,n];
    fclose(fid);
end

trial = 0;
figure
for i = sequence
    resultsSE3 = load(strcat('Data/test12_results/resultsSE3_',num2str(fibonacci(i)),'_1_1'));
    resultsSE3 = resultsSE3.resultsSE3;
    resultsNoSE3 = load(strcat('Data/test12_results/resultsNoSE3_',num2str(fibonacci(i)),'_1_1'));
    resultsNoSE3 = resultsNoSE3.resultsNoSE3;
    
    plot(edges(i-5),resultsSE3.ATE_translation_error,'*b')
    hold on;
    plot(edges(i-5),resultsSE3.ATE_rotation_error,'*g')
    hold on;
    plot(edges(i-5),resultsSE3.ASE_translation_error,'*m')
    hold on;
    plot(edges(i-5),resultsNoSE3.ATE_translation_error,'ob')
    hold on;
    plot(edges(i-5),resultsNoSE3.ATE_rotation_error,'og')
    hold on;
    plot(edges(i-5),resultsNoSE3.ASE_translation_error,'om')
    xlabel('nSE3MotionEdges')
    ylabel('absolute error values')
    legend('SE3 ATE','SE3 ARE','SE3 ASE','NoSE3 ATE','NoSE3 ARE',...
        'NoSE3 ASE','Location','northwest')
    title({SE3Vertex,strcat('Ternary factory noise level: ',noise)});
end
%print(strcat('Absolute_',num2str(trial)),'-dpdf')

figure
for i= sequence
    resultsSE3 = load(strcat('Data/test12_results/resultsSE3_',num2str(fibonacci(i)),'_1_1'));
    resultsSE3 = resultsSE3.resultsSE3;
    resultsNoSE3 = load(strcat('Data/test12_results/resultsNoSE3_',num2str(fibonacci(i)),'_1_1'));
    resultsNoSE3 = resultsNoSE3.resultsNoSE3;
    
    plot(edges(i-5),resultsSE3.AARPE_translation_error,'*c')
    hold on;
    plot(edges(i-5),resultsSE3.AARPE_rotation_error,'*k')
    hold on;
    plot(edges(i-5),resultsSE3.AARPTE_translation_error,'*r');
    hold on;
    plot(edges(i-5),resultsNoSE3.AARPE_translation_error,'oc')
    hold on;
    plot(edges(i-5),resultsNoSE3.AARPE_rotation_error,'ok')
    hold on;
    plot(edges(i-5),resultsNoSE3.AARPTE_translation_error,'or');
    xlabel('nSE3MotionEdges')
    ylabel('relative all-all error values')
    legend('SE3 allRTE','SE3 allRRE','SE3 allRSE','NoSE3 allRTE',...
        'NoSE3 allRRE','NoSE3 allRSE','Location','northwest');
    title({SE3Vertex,strcat('Ternary factory noise level: ',noise)});
end
%print(strcat('Relative_',num2str(trial)),'-dpdf')