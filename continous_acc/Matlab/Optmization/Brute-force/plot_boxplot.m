clear all;
load error_1.mat 
load error_2.mat 
load error_3.mat 
load error_4.mat 
load error_5.mat 
load error_6.mat 
load error_7.mat

error_CF = [e_CF_1 e_CF_2 e_CF_3 e_CF_4 e_CF_5 ...
                e_CF_6 e_CF_7];

error_mahony = [e_mahony_1 e_mahony_2 e_mahony_3 e_mahony_4 e_mahony_5 ...
                e_mahony_6 e_mahony_7];
            
error_madgwick = [e_madgwick_1 e_madgwick_2 e_madgwick_3 e_madgwick_4 e_madgwick_5 ...
    e_madgwick_6 e_madgwick_7];

positions = [1 1.15 1.3];
positions = [positions 0.75+positions 1.5+positions 2.25+positions 3+positions 3.75+positions 4.5+positions];


figure('position',[300 300 800 450]);
boxplot(error_mahony,'positions', positions);
set(gca,'xtick',[mean(positions(1:3)) mean(positions(4:6)) mean(positions(7:9))...
    mean(positions(10:12)) mean(positions(13:15)) mean(positions(16:18)) ...
    mean(positions(19:21))])
set(gca,'xticklabel',{'1','2','3','4','5','6','7'})
cc = lines(3);
color = repmat(cc,7,1);
h = findobj(gca,'Tag','Box');
for j=1:length(h)
   patch(get(h(j),'XData'),get(h(j),'YData'),color(j,:),'FaceAlpha',.5);
end
c = get(gca, 'Children');
hleg1 = legend(c(1:3), 'Yaw', 'Roll', 'Pitch','Orientation','Horizontal');
ylabel('Error [°]'); xlabel('Scenario'); title('Mahony error distribution');
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

figure('position',[300 300 800 450]);
boxplot(error_CF,'positions', positions);
set(gca,'xtick',[mean(positions(1:3)) mean(positions(4:6)) mean(positions(7:9))...
    mean(positions(10:12)) mean(positions(13:15)) mean(positions(16:18)) ...
    mean(positions(19:21))])
set(gca,'xticklabel',{'1','2','3','4','5','6','7'})
cc = lines(3);
color = repmat(cc,7,1);
h = findobj(gca,'Tag','Box');
for j=1:length(h)
   patch(get(h(j),'XData'),get(h(j),'YData'),color(j,:),'FaceAlpha',.5);
end
c = get(gca, 'Children');
hleg1 = legend(c(1:3), 'Yaw', 'Roll', 'Pitch','Orientation','Horizontal');
ylabel('Error [°]'); xlabel('Scenario'); title('CF error distribution')
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];

figure('position',[300 300 800 450]);
boxplot(error_madgwick,'positions', positions);
set(gca,'xtick',[mean(positions(1:3)) mean(positions(4:6)) mean(positions(7:9))...
    mean(positions(10:12)) mean(positions(13:15)) mean(positions(16:18)) ...
    mean(positions(19:21))])
set(gca,'xticklabel',{'1','2','3','4','5','6','7'})
cc = lines(3);
%color = ['c', 'y','r', 'c', 'y','r', 'c', 'y','r', 'c', 'y','r', 'c', 'y','r', 'c', 'y','r', 'c', 'y','r'];
color = repmat(cc,7,1);
h = findobj(gca,'Tag','Box');
for j=1:length(h)
   patch(get(h(j),'XData'),get(h(j),'YData'),color(j,:),'FaceAlpha',.5);
end
c = get(gca, 'Children');
hleg1 = legend(c(1:3), 'Yaw', 'Roll', 'Pitch','Orientation','Horizontal');
ylabel('Error [°]'); xlabel('Scenario'); title('Madgwick error distribution')
ax = gca;
outerpos = ax.OuterPosition;
ti = ax.TightInset; 
left = outerpos(1) + ti(1);
bottom = outerpos(2) + ti(2);
ax_width = outerpos(3) - ti(1) - ti(3);
ax_height = outerpos(4) - ti(2) - ti(4);
ax.Position = [left bottom ax_width ax_height];