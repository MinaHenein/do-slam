figure;
y = [results1LOW.ATE_translation_error,results1LOW.ATE_rotation_error,...
    results1LOW.ASE_translation_error;results1MED.ATE_translation_error,...
    results1MED.ATE_rotation_error,results1MED.ASE_translation_error;...
    results1HIGH.ATE_translation_error,results1HIGH.ATE_rotation_error,...
    results1HIGH.ASE_translation_error];
x = [0.01, 0.1, 1];
close all
semilogx(x,y,'-*','linewidth',2)
ax = gca;
ax.XTickLabel = x;
set(gcf,'color','w');
xlabel('std for 2-points-velocity edge')
ylabel('error values - low noise levels')
legend('ATE','ARE','ASE')

figure;
y = [results2LOW.ATE_translation_error,results2LOW.ATE_rotation_error,...
    results2LOW.ASE_translation_error;results2MED.ATE_translation_error,...
    results2MED.ATE_rotation_error,results2MED.ASE_translation_error;...
    results2HIGH.ATE_translation_error,results2HIGH.ATE_rotation_error,...
    results2HIGH.ASE_translation_error];
x = [0.01, 0.1, 1];
close all
semilogx(x,y,'-*','linewidth',2)
ax = gca;
ax.XTickLabel = x;
set(gcf,'color','w');
xlabel('std for 2-points-velocity edge')
ylabel('error values - high noise levels')
legend('ATE','ARE','ASE')