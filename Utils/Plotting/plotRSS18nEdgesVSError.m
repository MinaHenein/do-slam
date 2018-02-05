
k = [1,2,3,4,5,10,20];
edges = [];
for i = 1:length(k)
    j = 2;
    fid = fopen(strcat('Data/GraphFiles/RSS18g_measurements_',...
        num2str(k(i)),'_',num2str(j),'.graph'),'r');
    n = 0;
    tline = fgetl(fid);
    while ischar(tline)
        tline = fgetl(fid);
        n = n+1;
    end
    edges = [edges,n];
    fclose(fid);
end

for j = 3
    figure
    for i=1:1:length(k)
        resultsSE3 = load(strcat('Data/RSS18g_results/resultsSE3_',num2str(k(i)),'_',num2str(j)));
        resultsSE3 = resultsSE3.resultsSE3;              
        resultsNoSE3 = load(strcat('Data/RSS18g_results/resultsNoSE3_',num2str(k(i)),'_',num2str(j)));
        resultsNoSE3 = resultsNoSE3.resultsNoSE3;
        
        plot(edges(i),resultsSE3.ATE_translation_error,'*b')
        hold on;
        plot(edges(i),resultsSE3.ATE_rotation_error,'*g')
        hold on;
        plot(edges(i),resultsSE3.ASE_translation_error,'*m')
        hold on;
        plot(edges(i),resultsNoSE3.ATE_translation_error,'ob')
        hold on;
        plot(edges(i),resultsNoSE3.ATE_rotation_error,'og')
        hold on;
        plot(edges(i),resultsNoSE3.ASE_translation_error,'om')
        legend('SE3 ATE','SE3 ARE','SE3 ASE','NoSE3 ATE','NoSE3 ARE',...
            'NoSE3 ASE','Location','northwest')
    end
    figure
    for i=1:length(k)
        resultsSE3 = load(strcat('Data/RSS18g_results/resultsSE3_',num2str(k(i)),'_',num2str(j)));
        resultsSE3 = resultsSE3.resultsSE3;              
        resultsNoSE3 = load(strcat('Data/RSS18g_results/resultsNoSE3_',num2str(k(i)),'_',num2str(j)));
        resultsNoSE3 = resultsNoSE3.resultsNoSE3;
        
        plot(edges(i),resultsSE3.AARPE_translation_error,'*c')
        hold on;
        plot(edges(i),resultsSE3.AARPE_rotation_error,'*k')
        hold on;
        plot(edges(i),resultsSE3.AARPTE_translation_error,'*r');
        hold on;
        plot(edges(i),resultsNoSE3.AARPE_translation_error,'oc')
        hold on;
        plot(edges(i),resultsNoSE3.AARPE_rotation_error,'ok')
        hold on;
        plot(edges(i),resultsNoSE3.AARPTE_translation_error,'or');
        legend('SE3 allRTE','SE3 allRRE','SE3 allRSE','NoSE3 allRTE',...
            'NoSE3 allRRE','NoSE3 allRSE','Location','northwest');
        
    end
end
