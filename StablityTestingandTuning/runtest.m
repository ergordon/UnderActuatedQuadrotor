clear all
close all
Qo = diag([2,2,2,1,1,1]);
Ro = diag([1,1,1,1]);
Qi = diag([2,2,2,1,1,1]).^8;
Ri = diag([1,1,1,1]);
[T, X, Ui_T, Ui, Uo_T, Uo, odes] = simulate_this('Qo',Qo,'Ro',Ro,'Qi',Qi,'Ri',Ri);
plot_basic(T,X',Ui_T,Ui,Uo_T,Uo)

%%
close all;
Visualizer(T', X(:,1:3)', X(:,7:9)', odes', 'Test.mp4')
