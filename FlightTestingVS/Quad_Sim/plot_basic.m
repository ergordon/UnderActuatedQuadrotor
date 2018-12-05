figure(1)
hold on
plot(T,X(:,1),'b-');
plot(T,X(:,2),'g-');
plot(T,X(:,3),'r-');
title('Positions')
figure(2)
plot(T,X(:,4:6));
title('Velocities')
figure(3)
plot(T,X(:,7:9))
title('Angles')
figure(4)
plot(T,X(:,10:12))
title('Angular Rates')
% figure(5)
% plot(T,X(:,13:16))
% title('Motor Speeds')
% figure(6)
% plot(Ui_T,Ui)
% title('Inner Loop Control')
% figure(7)
% plot(Uo_T,Uo)
% title('Outer Loop Control')