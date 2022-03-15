
load("halvard.mat")

navn = "Halvard";
mn = mean(Lys);
st = std(Lys);

fig = figure;
% aktiver fig1
figure(fig);
clf(fig);

histogram(Lys,20);
subtitle(sprintf("%s, mean=%.1f std=%.1f",navn,mn,st));
xlabel('Lysmålinger');
ylabel('Antall målinger');
xlim([0 60])
ylim([0 15])