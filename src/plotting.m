%% Plotting
clear
close all

load('figData')
% Error

ePfSta.yaw = vnSta.yaw(1:end-1) - rad2deg(mekfSta.yaw(1:end-1))';
ePfSta.pitch = vnSta.pitch(1:end-1) - rad2deg(mekfSta.pitch(1:end-1))';
ePfSta.roll = vnSta.roll(1:end-1) - rad2deg(mekfSta.roll(1:end-1))';


ePfDS.yaw = vnDS.yaw(1:end-1) - rad2deg(mekfDS.yaw(1:end-2))';
ePfDS.pitch = vnDS.pitch(1:end-1) - rad2deg(mekfDS.pitch(1:end-2))';
ePfDS.roll = vnDS.roll(1:end-1) - rad2deg(mekfDS.roll(1:end-2))';

ePfDL.yaw = vnDL.yaw(1:end-1) - rad2deg(mekfDL.yaw(1:end-2)');
ePfDL.pitch = vnDL.pitch(1:end-1) - rad2deg(mekfDL.pitch(1:end-2))';
ePfDL.roll = vnDL.roll(1:end-1) - rad2deg(mekfDL.roll(1:end-2))';

rmseYaw = sqrt(sum(rmmissing(ePfSta.yaw).^2)/length(rmmissing(ePfSta.yaw)));
rmsePit = sqrt(sum(rmmissing(ePfSta.pitch).^2)/length(rmmissing(ePfSta.pitch)));
rmseRoll = sqrt(sum(rmmissing(ePfSta.roll).^2)/length(rmmissing(ePfDL.roll)));

%%
tSta = 0:(1/100):(1/100)*length(pfSta.yaw) - (1/100);

figure
subplot(3,1,1)
plot(tSta,vnSta.yaw)
hold on
plot(tSta,rad2deg(mekfSta.yaw))
plot(tSta(1:end-1),-1*rad2deg(madSta(:,1)))
plot(tSta,pfSta.yaw)
xlim([0 tSta(end)])
title('Yaw: Static Data')
ylabel('Yaw (degs)')
xlabel('Time (s)')
legend('VN Truth','MEKF','Madgwick','PF','Location','northwest')

subplot(3,1,2)
plot(tSta,vnSta.pitch)
hold on
plot(tSta,rad2deg(mekfSta.pitch))
plot(tSta(1:end-1),rad2deg(madSta(:,2)))
plot(tSta,pfSta.pitch)
xlim([0 tSta(end)])
title('Pitch: Static Data')
ylabel('Pitch (degs)')
xlabel('Time (s)')

subplot(3,1,3)
plot(tSta,vnSta.roll)
hold on
plot(tSta,rad2deg(mekfSta.roll))
plot(tSta(1:end-1),180+rad2deg(madSta(:,3)))
plot(tSta,pfSta.roll)
xlim([0 tSta(end)])
title('Roll: Static Data')
ylabel('Roll (degs)')
xlabel('Time (s)')

%%

tDS = 0:(1/20):(1/20)*length(pfDS.yaw) - (1/20);

figure
subplot(3,1,1)
plot(tDS(1:end-1),vnDS.yaw)
hold on
plot(tDS,rad2deg(mekfDS.yaw))
plot(tDS(1:end-1),rad2deg(madDS(:,1)))
plot(tDS,pfDS.yaw)
xlim([0 tDS(end)])
title('Yaw: Short Dynamic Data')
ylabel('Yaw (degs)')
xlabel('Time (s)')
legend('VN Truth','MEKF','Madgwick','PF','Location','northwest')

subplot(3,1,2)
plot(tDS(1:end-1),vnDS.pitch)
hold on
plot(tDS,rad2deg(mekfDS.pitch))
plot(tDS(1:end-1),rad2deg(madDS(:,2)))
plot(tDS,pfDS.pitch)
xlim([0 tDS(end)])
title('Pitch: Short Dynamic Data')
ylabel('Pitch (degs)')
xlabel('Time (s)')

subplot(3,1,3)
plot(tDS(1:end-1),vnDS.roll)
hold on
plot(tDS,rad2deg(mekfDS.roll))
plot(tDS(1:end-1),rad2deg(madDS(:,3)))
plot(tDS,pfDS.roll)
xlim([0 tDS(end)])
title('Roll: Short Dynamic Data')
ylabel('Roll (degs)')
xlabel('Time (s)')

%%

tDL = 0:(1/20):(1/20)*length(pfDL.yaw) - (1/20);

figure
subplot(3,1,1)
plot(tDL(1:end-1),vnDL.yaw)
hold on
plot(tDL,rad2deg(mekfDL.yaw))
plot(tDL(1:end-1),rad2deg(madDL(:,1)))
plot(tDL,pfDL.yaw)
xlim([0 tDL(end)])
title('Yaw: Long Dynamic Data')
ylabel('Yaw (degs)')
xlabel('Time (s)')
legend('VN Truth','MEKF','Madgwick','PF','Location','northwest')

subplot(3,1,2)
plot(tDL(1:end-1),vnDL.pitch)
hold on
plot(tDL,rad2deg(mekfDL.pitch))
plot(tDL(1:end-1),rad2deg(madDL(:,2)))
plot(tDL,pfDL.pitch)
xlim([0 tDL(end)])
title('Pitch: Long Dynamic Data')
ylabel('Pitch (degs)')
xlabel('Time (s)')

subplot(3,1,3)
plot(tDL(1:end-1),vnDL.roll)
hold on
plot(tDL,rad2deg(mekfDL.roll))
plot(tDL(1:end-1),rad2deg(madDL(:,3)))
plot(tDL,pfDL.roll)
xlim([0 tDL(end)])
title('Roll: Long Dynamic Data')
ylabel('Roll (degs)')
xlabel('Time (s)')

%%

figure
subplot(3,1,1)
plot(tSta,ePfSta.yaw)
title('Yaw Error: Short Data')
ylabel('Yaw Error (degs)')
xlabel('Time (s)')
xlim([0 tSta(end)])

subplot(3,1,2)
plot(tSta,ePfSta.pitch)
title('Pitch Error: Short Data')
ylabel('Pitch Error (degs)')
xlabel('Time (s)')
xlim([0 tSta(end)])

subplot(3,1,3)
plot(tSta,ePfSta.roll)
title('Roll Error: Short Data')
ylabel('Roll Error (degs)')
xlabel('Time (s)')
xlim([0 tSta(end)])

%%

figure
subplot(3,1,1)
plot(tDS(1:end-1),ePfDS.yaw)
title('Yaw Error: Short Dynamic Data')
ylabel('Yaw Error (degs)')
xlabel('Time (s)')
xlim([0 tDS(end)])

subplot(3,1,2)
plot(tDS(1:end-1),ePfDS.pitch)
title('Pitch Error: Short Dynamic Data')
ylabel('Pitch Error (degs)')
xlabel('Time (s)')
xlim([0 tDS(end)])

subplot(3,1,3)
plot(tDS(1:end-1),ePfDS.roll)
title('Roll Error: Short Dynamic Data')
ylabel('Roll Error (degs)')
xlabel('Time (s)')
xlim([0 tDS(end)])



%%

figure
subplot(3,1,1)
plot(tDL(1:end-1),ePfDL.yaw)
title('Yaw Error: Long Dynamic Data')
ylabel('Yaw Error (degs)')
xlabel('Time (s)')
xlim([0 tDL(end)])

subplot(3,1,2)
plot(tDL(1:end-1),ePfDL.pitch)
title('Pitch Error: Long Dynamic Data')
ylabel('Pitch Error (degs)')
xlabel('Time (s)')
xlim([0 tDL(end)])

subplot(3,1,3)
plot(tDL(1:end-1),ePfDL.roll)
title('Roll Error: Long Dynamic Data')
ylabel('Roll Error (degs)')
xlabel('Time (s)')
xlim([0 tDL(end)])
