A = readtable('Probe Plots- Exercise 2_task_3.xls');
%disp(A)

t = A{:,1};
x = A{:,2};
y = A{:,3};
z = A{:,4};

figure (1);
clf
plot (t,x);
xlabel ('time');
ylabel ('x');

figure (2);
clf
plot (t,y);
xlabel ('time');
ylabel ('y');

figure (3);
clf
plot (t,z);
xlabel ('time');
ylabel ('z');

figure (4);
clf
plot3 (x,y,z);
xlabel ('x');
ylabel ('y');
zlabel ('z');
