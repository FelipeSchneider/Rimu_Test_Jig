%https://stackoverflow.com/questions/15971478/most-efficient-way-of-drawing-grouped-boxplot-matlab
x = [1,2, 2,3, 4,3,6, 5,1,2, 2,3, 18,4,6];
group = [1,1,2,2,3,3,3,4,4,3,5,5,6,6,6];
positions = [1 1.15 1.3 2 2.15 2.3];
boxplot(x,group, 'positions', positions);

set(gca,'xtick',[mean(positions(1:3)) mean(positions(4:6)) ])
set(gca,'xticklabel',{'Direct care','Housekeeping'})

color = ['c', 'y','r', 'c', 'y','r'];
h = findobj(gca,'Tag','Box');
for j=1:length(h)
   patch(get(h(j),'XData'),get(h(j),'YData'),color(j),'FaceAlpha',.5);
end

c = get(gca, 'Children');

hleg1 = legend(c(1:3), 'Feature1', 'Feature2', 'Feature3' );


