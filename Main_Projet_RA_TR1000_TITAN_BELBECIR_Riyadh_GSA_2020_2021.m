clear 
close all hidden
clc


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%                                                                     %%
%%%% Remarque :  - Le script permet de planifier la trajectoire du robot %%
%%%%             KUKA KR1000 TITAN                                       %%
%%%%             - Dans les lignes 28 29 30 ou bien 35 36 37  :          %%     
%%%%             - Entrez : votre configuration angulaire de départ Qin  %%
%%%%                        Votre configuration angulaire d'arrivée Qf   %%
%%%%                        votre Temps d'arivvée désiré TimeF           %%
%%%%                                                                     %%
%%%%             - N'oubliez pas de taper un bouton du clavier lorsque   %%
%%%%               la premiere figure s'affiche.                         %%
%%%%                                                                     %%
%%%%                                                  Enjoy ;) !!!       %%
%%%%                                                                     %%
%%%%                  Made By : BELBECIR Riyadh Abbes GSA 2020 2021      %%
%%%%                            Univérsité Claude Bernard Lyon 1         %%
%%%%                            riyadh-belbecir@etu.univ-lyon1.fr        %%
%%%%                                                                     %%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 
%%%% Trajectoire qui dépasse les limites physique du robot 

%Qin = [0 0 0 0 pi/2 pi/4]; % La configuration de départ ( Nulle C.A.D configuration de reference est pareil au schema cinématique)
%Qf = [1*pi/2 0 4*pi/3 0*pi pi/4 pi/4];% La configuration d'arrivée
%TimeF = 10 ; % Temps d'arrivée


%%%% Trajectoire qui respecte les limites physiques du robot

Qin = [0 0 0 0 pi/2 pi/4]; % La configuration de départ ( Nulle C.A.D configuration de reference est pareil au schema cinématique)
Qf = [1*pi/2 0 +3.0 0*pi pi/4 pi/4];% La configuration d'arrivée
TimeF = 10 ; % Temps d'arrivée



figure('units','normalized','outerposition',[0 0 1 1])% Option d'affichage
figure(1) %Option d'affichage
syms T1 T2 T3 T4 T5 T6; % Les angles symboliques du notre bras robotique.
T =[T1 T2 T3 T4 T5 T6]; % Le Vecteur Theta's

% La configuration du robot initiale est identique à celle 
% utilisée pour le schéma cinématique

Tin = [+pi/2 +pi/2 0 pi pi 0]'; % Vecteur de l'initialisation des angles.
     %[+90   +90   0 180 180 0]';

% La Matrice DH qui contient le tableau des parametres Denavit-Hartenberg
% du notre robot KUKA KR1000 Titan
     
DH = [+600 +pi/2 1100 T(1)+Tin(1);...
    +1400 0 0 T(2)+Tin(2);...
    +65 +pi/2 0 T(3)+Tin(3);...
    0 +pi/2 +650+550 T(4)+Tin(4);...
    0 +pi/2 0 T(5)+Tin(5);...
    0 0 +372 T(6)+Tin(6)];


MGD = eye(4); % L'initialisation de la multiplication a 1(un)
Joints_Ends = []; % Les positions des extrémités des liens par rapport au repere fixe
% Joints_Ends : Utile pour le dessin du robot et l'animation
% J'ai pas utilisé les orientations des liens pour simplifier le Programme (ce n'est pas le but).
for i = 1 : length(T)
    % Les matrices de transformation homogenes pour chaque 2 reperes succesives.
    L = [cos(DH(i,4)) -cos(DH(i,2))*sin(DH(i,4)) sin(DH(i,2))*sin(DH(i,4)) DH(i,1)*cos(DH(i,4));...
        sin(DH(i,4)) cos(DH(i,2))*cos(DH(i,4)) -sin(DH(i,2))*cos(DH(i,4)) DH(i,1)*sin(DH(i,4));...
        0 sin(DH(i,2)) cos(DH(i,2)) DH(i,3);...
        0 0 0 1]; 
    
    MGD = MGD * L; % Le modele Geometrique Direct
    Joints_Ends = [Joints_Ends MGD(1:3,4)]; % Option d'affichage
end


% La planification de la trajectoire dans l'espace articulaire

Ts = 50; %Time samples : la période d'echantillonage

QinD = Qin*180/pi
%Qf = [0.7 -0.4 10 0 1.5 0.5];

QfD = Qf*180/pi
DeltaQ = Qf - Qin ;% Constante répétitive dans les formules 
TimeI = 0 ; % Temps de départ


t  = TimeI : (TimeF-TimeI)/Ts : TimeF; % Le vecteur Temps


%
% q : Positions Ang ; qdot : Vitesses Ang ; q2dot : Accelerations Ang  dim : 6xlength(t) 
q =[]; 
qdot = [];
q2dot = [];
for i = 1:length(DeltaQ)
q = [q ; Qin(i)+(10*DeltaQ(i)/(TimeF^3))*t.^3-(15*DeltaQ(i)/(TimeF^4))*t.^4+(6*DeltaQ(i)/(TimeF^5))*t.^5];
qdot =[qdot;(30*DeltaQ(i)/(TimeF^3))*t.^2-(60*DeltaQ(i)/(TimeF^4))*t.^3+(30*DeltaQ(i)/(TimeF^5))*t.^4];
q2dot = [q2dot;60*DeltaQ(i)/(TimeF^3)*t-180*DeltaQ(i)/(TimeF^4)*t.^2+120*DeltaQ(i)/(TimeF^5)*t.^3];
end



% La planification d'une trajctoire rectiligne dans l'espace operationnel
% Elle va nous servir pour faire des comparaisons avec l'espace articulaire

P_d = MGD;  % P_d : Point de départ (Position de l'effecteur par rapport au repere fixe) 
P_a = MGD;  % P_a : Point d'arrivée

% L'extraction des positions de départ et d'arrivée à partir du MGD 
% c'est l'intersection de la quaterieme colonne et les trois premieres lignes
for i =1 : 6 
    % Substitution par les valeurs numériques dans le model Symbolic
   P_d =  subs(P_d,T(i),Qin(i)); 
   P_a =  subs(P_a,T(i),Qf(i));
end
P_d = double(P_d(1:3,4)); % L'extraction
P_a = double(P_a(1:3,4));

% La définition de la droite operationnelle 
% Définir les vecteurs de la droite selon X,Y,Z (Projections) Pour que je puisse déssiner la
% rectiligne au fur et à mesure en animation avec la trajectoire
% articulaire 
%(Le nombre des echantillons de la trajectoire articulaire = Nombre des Ech Rectiligne)

if((P_d(1)-P_a(1))>0)
Line_Operi = P_d(1) : -abs(P_d(1)-P_a(1))/Ts : P_a(1);
elseif((P_d(1)-P_a(1))<0)
Line_Operi = P_d(1) : +abs(P_d(1)-P_a(1))/Ts : P_a(1);
elseif((P_d(1)-P_a(1))==0)
Line_Operi =zeros(1,Ts+1)+P_d(1); % Projection de la droite sur X
end

if((P_d(2)-P_a(2))>0)
Line_Operj = P_d(2) : -abs(P_d(2)-P_a(2))/Ts : P_a(2);
elseif((P_d(2)-P_a(2))<0)
Line_Operj = P_d(2) : +abs(P_d(2)-P_a(2))/Ts : P_a(2);
elseif((P_d(2)-P_a(2))==0)
  Line_Operj =zeros(1,Ts+1)+P_d(2); % Projection de la droite sur Y
end

if((P_d(3)-P_a(3))>0)
Line_Operk = P_d(3) : -abs(P_d(3)-P_a(3))/Ts : P_a(3);
elseif((P_d(3)-P_a(3))<0)
Line_Operk = P_d(3) : +abs(P_d(3)-P_a(3))/Ts : P_a(3);
elseif((P_d(3)-P_a(3))==0)
Line_Operk =zeros(1,Ts+1)+P_d(3); % Projection de la droite sur Z
end

Line_Oper =[Line_Operi;Line_Operj;Line_Operk]; % Les coordonnées finales de la droite 


Trajectory = [];% Pour La trajectoire Articulaire de l'effecteur



finale = [0;0;0]; % La matrice qui sera utilisé pour enregistré 
for i = 1:6
 finale = [finale +Joints_Ends(:,i)];% La Matrice qui contient les postions des joints du robot
 % Elle est en symbolic C.A.D général pour n'importe qu'elle cas 
 % Par la suite les extrémités vont etre calculer et substituer dans un
 % boucle en fonction de q instantané.
end
%finalenum = finale;
%MGDnum = MGD;


% La boucle qui compile tous les calcules précédents
% - Le calcule des angles et des positions des liens en fonction des trajectoires prédefinies 
%   au cour du temps (à chaque itération) 
% - le traçage du robot et son animation (Articulaire)
% - Le traçage dela trajectoire rectiligne (Operationnel)
for j = 1 :length(q(1,:))
Tnum = [q(1,j) q(2,j) q(3,j) q(4,j) q(5,j) q(6,j)]; % Les angles définies par la trajectoire
finalenum = finale; % Les positions des liens
                    % les coordonnées de l'effecteur par rapport au repere fixe 
                    % sont situer sur la derniere colonne de ce vecteur.
                    % A chaque iteration la valeur Num est reinitialisée en
                    % Sym 

    
for i =1 : 6 % La substitutions des valeurs Num pour Syms
    finalenum = subs(finalenum,T(i),Tnum(i)); 
    % Cette commande substitue la variable Sym T par La valeur Num Tnum
    % dans les modele 
end

plot3(finalenum(1,1:2), finalenum(2,1:2),finalenum(3,1:2), 'b', 'LineWidth', 4);
hold on;
plot3(finalenum(1,2:3), finalenum(2,2:3),finalenum(3,2:3), 'r', 'LineWidth', 4);
hold on;
plot3(finalenum(1,3:4), finalenum(2,3:4),finalenum(3,3:4), 'g', 'LineWidth', 4);
hold on;
plot3(finalenum(1,4:5), finalenum(2,4:5),finalenum(3,4:5), 'y', 'LineWidth', 4);
hold on;
plot3(finalenum(1,5:6), finalenum(2,5:6),finalenum(3,5:6), 'k', 'LineWidth', 4);
hold on;
plot3(finalenum(1,6:7), finalenum(2,6:7),finalenum(3,6:7), 'm', 'LineWidth', 4);
hold on
plot3([-3637 +3637],[0,0],[0,0],'LineWidth', 4)
hold on
plot3([0 0],[-3637 +3637],[0,0],'LineWidth', 4)
hold on
plot3([0 0],[0,0],[-4137 +4137],'LineWidth', 4)
xlabel('Laxe X')
ylabel('Laxe Y')
zlabel('Laxe Z')
axis([-3637 +3637 -3637 +3637 -4137 +4137])
title('KUKA - KR1000 Titan')
box on
ax = gca;
ax.ZGrid = 'on';
ax.XGrid = 'on';
ax.YGrid = 'on';
drawnow update
if j ==1
        %hold on
 %  hh = sgtitle('Appuyer sur un boutton du clavier pour lancer la simulation');% suptitle('Appuyer sur un boutton du clavier pour lancer la simulation');
  %  set(hh,'Color','red')
       an = annotation('textbox', [0, 1, 0.1, 0.0], 'string', 'Appuyer sur un boutton du clavier pour lancer la simulation');
    set(an,'Color','red','FontSize',12)
    pause 
  % hhh = sgtitle('Enjoy ;) !!');%suptitle('Enjoy ;) !!');
 %  set(hhh,'Color','green')
 delete(an);
   an = annotation('textbox', [0, 1, 0.1, 0.0], 'string', 'Enjoy ;) !!');
    set(an,'Color','green','FontSize',12)
end
hold off
if j == length(q(1,:))
    hold on 
end
Trajectory = [Trajectory [finalenum(1,7), finalenum(2,7),finalenum(3,7)]']; % Enregistrement de la trajectoire de l'effecteur
%if j ~= 1
plot3(Trajectory(1,1:j),Trajectory(2,1:j),Trajectory(3,1:j)) % Affichage de la trajectoire articulaire
hold on
plot3(Line_Operi(1,1:j),Line_Operj(1,1:j),Line_Operk(1,1:j),'r') % Affichage de la droite operationnelle
hold on
%end
%hold on
%view([q(1,j) 10*j 100])
%pause(0.00001)
%rotate3d on
end
rotate3d on
Error = abs(Trajectory - Line_Oper); % La comparaison entre les deux planifications

Speed_Limits = [58 50 50 60 60 84]*pi/180;


%%%%%%%%%    Les Angles Limites De Notre Robot %%%%%%%%%%%%
 T1bound = 150*pi/180+0;
 T2bound_pos =40*pi/180;%+pi/2;
 T2bound_neg =-107.5*pi/180;%+pi/2;
 T3bound_pos =200*pi/180+0;
 T3bound_neg =-55*pi/180+0;
 T4bound =350*pi/180;%+pi/2;
 T5bound =118*pi/180;%+pi/2;
 T6bound =350*pi/180+0;
 Upper_Bound =[T1bound T2bound_pos T3bound_pos T4bound T5bound T6bound];
 Lower_Bound =[-T1bound T2bound_neg T3bound_neg -T4bound -T5bound -T6bound];
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ce qui suit c'est l'affichage des trajectoires angulaires, les vitesses
% angulaires , les accélérations angulaires ,La comparaison entre les deux
% planifications
figure('units','normalized','outerposition',[0 0 1 1])
figure(2)

subplot(2,3,1)
plot(t,q(1,:))
hold on
plot(t,T1bound*ones(1,length(t)),'r','LineWidth',3)
hold on
plot(t,-T1bound*ones(1,length(t)),'r','LineWidth',3)
xlabel('Time (s)')
ylabel('Angular Position (rad)')
title('Theta 1')
subplot(2,3,2)
plot(t,q(2,:))
hold on
plot(t,T2bound_pos*ones(1,length(t)),'r','LineWidth',3)
hold on
plot(t,T2bound_neg*ones(1,length(t)),'r','LineWidth',3)
xlabel('Time (s)')
ylabel('Angular Position (rad)')
title('Theta 2')
subplot(2,3,3)
plot(t,q(3,:))
hold on
plot(t,T3bound_pos*ones(1,length(t)),'r','LineWidth',3)
hold on
plot(t,T3bound_neg*ones(1,length(t)),'r','LineWidth',3)
xlabel('Time (s)')
ylabel('Angular Position (rad)')
title('Theta 3')
subplot(2,3,4)
plot(t,q(4,:))
hold on
plot(t,T4bound*ones(1,length(t)),'r','LineWidth',3)
hold on
plot(t,-T4bound*ones(1,length(t)),'r','LineWidth',3)
xlabel('Time (s)')
ylabel('Angular Position (rad)')
title('Theta 4')
subplot(2,3,5)
plot(t,q(5,:))
hold on
plot(t,T5bound*ones(1,length(t)),'r','LineWidth',3)
hold on
plot(t,-T5bound*ones(1,length(t)),'r','LineWidth',3)
xlabel('Time (s)')
ylabel('Angular Position (rad)')
title('Theta 5')
subplot(2,3,6)
plot(t,q(6,:))
hold on
plot(t,T6bound*ones(1,length(t)),'r','LineWidth',3)
hold on
plot(t,-T6bound*ones(1,length(t)),'r','LineWidth',3)
xlabel('Time (s)')
ylabel('Angular Position (rad)')
title('Theta 6')
sgtitle('Les Trajectoires Angulaires')%suptitle('Les Trajectoires Angulaires')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


figure('units','normalized','outerposition',[0 0 1 1])
figure(3)

subplot(2,3,1)
plot(t,qdot(1,:))
hold on
plot(t,Speed_Limits(1)*ones(1,length(t)),'r','LineWidth',3)
hold on
plot(t,-Speed_Limits(1)*ones(1,length(t)),'r','LineWidth',3)
xlabel('Time (s)')
ylabel('Angular Speed (rad/s)')
title('Theta 1')
subplot(2,3,2)
plot(t,qdot(2,:))
hold on
plot(t,Speed_Limits(2)*ones(1,length(t)),'r','LineWidth',3)
hold on
plot(t,-Speed_Limits(2)*ones(1,length(t)),'r','LineWidth',3)
xlabel('Time (s)')
ylabel('Angular Speed (rad/s)')
title('Theta 2')
subplot(2,3,3)
plot(t,qdot(3,:))
hold on
plot(t,Speed_Limits(3)*ones(1,length(t)),'r','LineWidth',3)
hold on
plot(t,-Speed_Limits(3)*ones(1,length(t)),'r','LineWidth',3)
xlabel('Time (s)')
ylabel('Angular Speed (rad/s)')
title('Theta 3')
subplot(2,3,4)
plot(t,qdot(4,:))
hold on
plot(t,Speed_Limits(4)*ones(1,length(t)),'r','LineWidth',3)
hold on
plot(t,-Speed_Limits(4)*ones(1,length(t)),'r','LineWidth',3)
xlabel('Time (s)')
ylabel('Angular Speed (rad/s)')
title('Theta 4')
subplot(2,3,5)
plot(t,qdot(5,:))
hold on
plot(t,Speed_Limits(5)*ones(1,length(t)),'r','LineWidth',3)
hold on
plot(t,-Speed_Limits(5)*ones(1,length(t)),'r','LineWidth',3)
xlabel('Time (s)')
ylabel('Angular Speed (rad/s)')
title('Theta 5')
subplot(2,3,6)
plot(t,qdot(6,:))
hold on
plot(t,Speed_Limits(6)*ones(1,length(t)),'r','LineWidth',3)
hold on
plot(t,-Speed_Limits(6)*ones(1,length(t)),'r','LineWidth',3)
xlabel('Time (s)')
ylabel('Angular Speed (rad/s)')
title('Theta 6')
sgtitle('Les Vitesses Angulaires')%suptitle('Les Vitesses Angulaires')


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



figure('units','normalized','outerposition',[0 0 1 1])
figure(4)

subplot(2,3,1)
plot(t,q2dot(1,:))
xlabel('Time (s)')
ylabel('Angular Acceleration (rad/s²)')
title('Theta 1')
subplot(2,3,2)
plot(t,q2dot(2,:))
xlabel('Time (s)')
ylabel('Angular Acceleration (rad/s²)')
title('Theta 2')
subplot(2,3,3)
plot(t,q2dot(3,:))
xlabel('Time (s)')
ylabel('Angular Acceleration (rad/s²)')
title('Theta 3')
subplot(2,3,4)
plot(t,q2dot(4,:))
xlabel('Time (s)')
ylabel('Angular Acceleration (rad/s²)')
title('Theta 4')
subplot(2,3,5)
plot(t,q2dot(5,:))
xlabel('Time (s)')
ylabel('Angular Acceleration (rad/s²)')
title('Theta 5')
subplot(2,3,6)
plot(t,q2dot(6,:))
xlabel('Time (s)')
ylabel('Angular Acceleration (rad/s²)')
title('Theta 6')
sgtitle('Les Accelerations Angulaires')%suptitle('Les Accelerations Angulaires')



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure('units','normalized','outerposition',[0 0 1 1])
figure(5)

subplot(3,1,1)
plot(t,Error(1,:))
xlabel('Time (s)')
ylabel('X Error (rad)')
title('erreur sur l axe X --> (Articulaire - Operationnel)')
subplot(3,1,2)
plot(t,Error(2,:))
xlabel('Time (s)')
ylabel('Y Error (rad)')
title('erreur sur l axe Y --> (Articulaire - Operationnel)')
subplot(3,1,3)
plot(t,Error(3,:))
xlabel('Time (s)')
ylabel('Z Error (rad)')
title('erreur sur l axe Z --> (Articulaire - Operationnel)')
sgtitle('La comparaison entre Lespace Articulaire et Operationnel')
%suptitle('La comparaison entre Lespace Articulaire et Operationnel')

%%%%%%%%%    Les Angles Limites De Notre Robot %%%%%%%%%%%%%%%%%%
 
 %%%%%%%%        Les vitesses Limites          %%%%%%%%%%%%%%%%%%
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

 
 %%%%%%%% Verification de l'existence du robot dans la marge des angles limites.%%%%%%%%%%%%%%%%%%%%

% for (i = 1:length(q(1,:)))
% if (q(1,i)>Upper_Bound(1) || q(2,i)>Upper_Bound(2) || q(3,i)>Upper_Bound(3) || q(4,i)>Upper_Bound(4) || q(5,i)>Upper_Bound(5) || q(6,i)>Upper_Bound(6))
% error('\nInitial Position out of range .\n\n Angles should be in Radians as follow \n\n T1 = +150°/-150°\n T2 = +40°/-107.5° \n T3 = +20°/-235° \n T4 = +350°/-350° \n T5 = +118°/-118° \n T6 = +350°/-350° ',class(i))
% elseif(q(1,i)<Lower_Bound(1) || q(2,i)<Lower_Bound(2)|| q(3,i)<Lower_Bound(3)|| q(4,i)<Lower_Bound(4)|| q(5,i)<Lower_Bound(5)|| q(6,i)<Lower_Bound(6))
% error('Initial Position out of range .  \n\n T1 = +150°/-150°\n T2 = +40°/-107.5° \n T3 = +20°/-235° \n T4 = +350°/-350° \n T5 = +118°/-118° \n T6 = +350°/-350° ',class(i))
% end
% end

Alerte = 0;
for i = 1 : length(q(1,:))
if (q(1,i)<T1bound && q(1,i)>-T1bound)
    elseif (q(1,i)<0)
    disp('lower bound 1')  %% Un message qui nous indique qu'elle limite est atteinte
    Alerte = Alerte + 1;
    elseif (q(1,i)>0)
    disp('upper bound 1')
    Alerte = Alerte + 1;
end

if (q(2,i)<T2bound_pos && q(2,i)>T2bound_neg)
    
    elseif (q(2,i)<0)
    disp('lower bound 2')
    Alerte = Alerte + 1;
    elseif (q(2,i)>0)
    disp('upper bound 2')
    Alerte = Alerte + 1;
end

if (q(3,i)<T3bound_pos && q(3,i)>T3bound_neg)
    
    elseif (q(3,i)<0)
    disp('lower bound 3')
    Alerte = Alerte + 1;
    elseif (q(3,i)>0)
  disp('upper bound 3');
 
    Alerte = Alerte + 1;
end


if (q(4,i)<+T4bound && q(4,i)>-T4bound)
    
    elseif (q(4,i)<0)
    disp('lower bound 4')
    Alerte = Alerte + 1;
    elseif (q(4,i)>0)
    disp('upper bound 4')
    Alerte = Alerte + 1;
end


if (q(5,i)<+T5bound && q(5,i)>-T5bound)
    
    elseif (q(5,i)<0)
    disp('lower bound 5')
    Alerte = Alerte + 1;
    elseif (q(5,i)>0)
    disp('upper bound 5')
    Alerte = Alerte + 1;
end
if (q(6,i)<+T6bound && q(6,i)>-T6bound)
    
    elseif (q(6,i)<0)
    disp('lower bound 6')
    Alerte = Alerte + 1;
    elseif (q(6,i)>0)
    disp('upper bound 6')
    Alerte = Alerte + 1;
end
end
Alerte_Speed = 0;
      if(Speed_Limits(1)<max(abs(qdot(1,:))))
         disp('Vitesse maximale dépassé 1')
         Alerte_Speed = Alerte_Speed + 1;
     end
     if(Speed_Limits(2)<max(abs(qdot(2,:))))
         disp('Vitesse maximale dépassé 2')
         Alerte_Speed = Alerte_Speed + 1;
     end
     if(Speed_Limits(3)<max(abs(qdot(3,:))))
         disp('Vitesse maximale dépassé 3')
         Alerte_Speed = Alerte_Speed + 1;
     end
     if(Speed_Limits(4)<max(abs(qdot(4,:))))
         disp('Vitesse maximale dépassé 4')
     end
     if(Speed_Limits(5)<max(abs(qdot(5,:))))
         disp('Vitesse maximale dépassé 5')
         Alerte_Speed = Alerte_Speed + 1;
     end
     if(Speed_Limits(6)<max(abs(qdot(6,:))))
         disp('Vitesse maximale dépassé 6')
         Alerte_Speed = Alerte_Speed + 1;
     end
     
     
if Alerte ~=0 
warning('Le Robot est dans une Zone Interdite (singularité)')
myicon = imread('Warning_W.jpg');
h=msgbox({'Robot is Working out of angles range';'';'   Verify your Trajectory ';'';'T1 = +150°/-150°'; 'T2 = +40°/-107.5°';'T3 = +200°/-55°';'T4 = +350°/-350°';'T5 = +118°/-118°';'T6 = +350°/-350°'},'Warning','custom',myicon);
end

if Alerte_Speed ~=0 
warning('L actionneur ne peut pas fournir les vitesses demandées. Allongez le temps final')
h=msgbox({'Robot Maximum Speed Exceeded';' Change Finale_Time ';'';'Speed_Limits = [58 50 50 60 60 84]°*pi/180 rad/s'},'Warning','custom',myicon);
end