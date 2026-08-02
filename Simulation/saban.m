% Drawing mapline
function saban()
    clf;
    format short;
    hold on;
    axis equal;
    grid on;
    
    %Ve duong thang AB
    Xab = linspace(0,-2500,1000);
    Yab = Xab*0;
    plot(Xab,Yab,'k','LineWidth',2);
    rectangle('Position', [-2150 -150 300 300], 'EdgeColor', 'r', 'LineWidth', 1, 'LineStyle', '--');
    th = linspace( pi/2, 3*pi/2, 1000);
    R = 500;
    x = R*cos(th) - 2500;
    y = R*sin(th) + 500;
    plot(x,y,'k','LineWidth',2);
    
    % Ve duong thang DE
    Xde = linspace(-2500,-2331.3708,1000);
    Yde = 1000 + Xde*0;
    plot(Xde,Yde,'k','LineWidth',2);
        
    % RED   
    % Ve duong cong EF
    Xef = linspace(-2331.3708,-1765.6854,1000);
    Yef = 1800 - sqrt(800^2 - (Xef + 2331.3708).^2);
    plot(Xef,Yef,'k','LineWidth',2,'color',[1 0 0]);
    
    % Ve duong cheo FH
    Xfh = linspace(-1765.6854,-1734.3146,1000);
    Yfh = linspace(1234.3146,1265.6854,1000);
    plot(Xfh,Yfh,'k','LineWidth',2,'color',[1 0 0]);
        
    % Ve duong cong HI
    Xhi = linspace(-1734.3146,-1168.6292,1000);
    Yhi = 700 + sqrt(800^2 - (Xhi + 1168.6292).^2);
    plot(Xhi,Yhi,'k','LineWidth',2,'color',[1 0 0]);
        
    % Ve duong thang IJ
    Xij = linspace(0,-1168.6292,1000);
    Yij = 1500 + Xij*0;
    plot(Xij,Yij,'k','LineWidth',2,'color',[1 0 0]);
     
    % BLUE
    % Ve duong cong EG
    Xeg = linspace(-2331.3708,-1765.6854,1000);
    Yeg = 200 + sqrt(800^2 - (Xeg + 2331.3708).^2);
    plot(Xeg,Yeg,'k','LineWidth',2,'color',[0 0 1]);
    
    % Ve duong cheo GK
    Xgk = linspace(-1765.6854,-1734.3146,1000); 
    Ygk = linspace(1265.6854,1234.3146,1000) - 500;
    plot(Xgk,Ygk,'k','LineWidth',2,'color',[0 0 1]);
     
    % Ve duong cong KL
    Xkl = linspace(-1765.6854,-1168.6292,1000);
    Ykl = 1300 - sqrt(800^2 - (Xkl + 1168.6292).^2);
    plot(Xkl,Ykl,'k','LineWidth',2,'color',[0 0 1]);
        
    % Ve duong thang LM
    Xlm = linspace(0,-1168.6292,1000);
    Ylm = 500 + Xlm*0;
    plot(Xlm,Ylm,'k','LineWidth',2,'color',[0 0 1]);
    
    grid on;
    xlabel('X(mm)')
    ylabel('Y(mm)')
    
end