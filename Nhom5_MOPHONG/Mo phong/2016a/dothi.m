Mo_phong
sim('Mo_phong');
a=(x+0.2395*sin(goc));
b=(y+0.2395*cos(goc));
p1=plot(a-0.051*cos(goc),b+0.051*sin(goc),'r');
hold on
plot(a+0.051*cos(goc),b-0.051*sin(goc),'r');
p2=plot(x,y);
p3=plot([0 0],[0 1.5],'m');
fplot(@(x) 1.5+0.5-0.5/5^(6*x),[0 x(length(x))],'m');
axis equal
legend([p1,p2,p3],{'Quy dao cua hai cam bien ngoai cung','Quy dao tam xe','Duong line'});
xlabel('x (m)');
ylabel('y (m)');