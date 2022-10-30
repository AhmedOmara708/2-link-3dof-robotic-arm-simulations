A = readtable('Probe Plots- Exercise 2.xls');
%disp(A)

t = A{:,1};
x = A{:,2};
y = A{:,3};
z = A{:,4};

figure (1);
clf
plot (t,x);
hold on
plot (t,y);
hold on
plot (t,z);
xlabel ('time');
ylabel ('coordinate');
legend('x','y','z');

figure (4);
clf
plot3 (x,y,z);
xlabel ('x');
ylabel ('y');
zlabel ('z');
