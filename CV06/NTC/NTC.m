clear all;
close all;
tcsv = csvread('ntc.csv');
ad = tcsv(:,1);
t = tcsv(:,2);

figure(1);
plot(ad,t);
title('Závislost odporu NTC na teplotě');

figure(2);
t =1023*t./(10*ones(size(t))+t);
ad2 = 0:1023;
plot(t,ad,'o');

p = polyfit(t, ad, 10);

ad2 = 0:1023;
t2 = round(polyval(p, ad2), 1);
hold on, plot(ad2, t2, 'r'); 
title('vztah hodnoty ADC a teploty');
dlmwrite('data.dlm', t2*10, ','); 