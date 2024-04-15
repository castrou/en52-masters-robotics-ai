function plot_robot(r)
    hold on; grid on; axis equal;        
    %axis([r(1)-5 r(1)+5 r(2)-5 r(2)+5])   
    i = [0.1;0;1];
    j = [0;0.1;1];
    p = [0.1;0.1;1];
    c = cos(r(3));
    s = sin(r(3));
    t = [r(1);r(2)];
    T = [c -s t(1);s c t(2); 0 0 1];
    ir = T*i;
    jr = T*j;
    pg =T*p;
    %scatter(pg(1),pg(2),'ko')
    plot([0,i(1)],[0,i(2)],'r');
    plot([0,j(1)],[0,j(2)],'b');
    plot([r(1),ir(1)],[r(2),ir(2)],'r');
    plot([r(1),jr(1)],[r(2),jr(2)],'b');