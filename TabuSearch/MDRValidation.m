clear all;
close all;

% Set Style
cal_orange = [237 125 49]/255;
cal_blue = [50 142 216]/255;
cal_grey_light = [0.95 0.95 0.95];
cal_grey_dark = [0.35 0.35 0.35];

allpoints = readtable('AllPoints.csv');
pareto = readtable('Points.csv');

obj1 = table2array(allpoints(:,2))';
obj2 = table2array(allpoints(:,3))';

obj1_pareto = table2array(pareto(:,2))';
obj2_pareto = table2array(pareto(:,3))';

figure(1)
% set figure position and size:
set(gcf,'position',[160 200 500 400])

% keep position and size when printing:
set(gcf,'PaperPositionMode','auto')

hold on
scatter(obj1,obj2,50,'x','LineWidth',1.2,'DisplayName','Candidate Designs');
plot(obj1_pareto,obj2_pareto,'Marker','.','MarkerSize',20,'color',cal_orange,...
    'LineWidth',1.5,'DisplayName','Pareto Front');
hold off

% xlim([-18 -6]);
% xticks(linspace(-18,-6,3));
% ylim([4e3 32e3]);
% yticks(linspace(4e3, 32e3, 3));

xlabel('Objective 1','interpreter','latex');
ylabel('Objective 2');

leg = legend('location','northoutside','orientation','horizontal');

% set fonts and frame:
set(gca,'Fontn','Times','FontSize',18,'linewidth',1)
  

% % For a vectorial figure (for latex):
% print -deps2c MDR_Validation.eps
% 
% % For an image figure at resolution 300 pixels/inch (for word):
% print -dpng -r300 MDR_Validation.png
% 
