clear 
close all
clc
figure('units','normalized','outerposition',[0 0 1 1])

%%%%%% Le Programme est fonctionnel, il suffit de le lancer 
%%%%%% Les variables ne sont pas identiques à celles du programme principal
%%%%%% Ce script n'est pas final.

Tin =01*[+01*2.44+0*1 0*-pi/2-0.3 0*1 0*1 0*1 0*1]; %Le vecteur des angles initiales.
Tf = 1*[pi/2-Tin(1) +pi/2-Tin(2) 0-Tin(3) pi-Tin(4) pi-Tin(5) 0-Tin(6)]'; % Le vecteur des angles désirés (Finales)


% A = [0 1;0 0];
% B = [0;1];
% C = [1 0];
% D = 0;
% Sys = ss(A,B,C,D);


%%%%%%%%%    Les Angles Limites Du Notre Robot %%%%%%%%%%%%
 T1bound = 150*pi/180+pi/2%;
 T2bound_pos =40*pi/180+pi/2%;
 T2bound_neg =-107.5*pi/180+pi/2%;
 T3bound_pos =20*pi/180+0%;
 T3bound_neg =-235*pi/180+0%;
 T4bound =350*pi/180+pi/2%;
 T5bound =118*pi/180+pi/2%;
 T6bound =350*pi/180+0%;
 Upper_Bound =[T1bound T2bound_pos T3bound_pos T4bound T5bound T6bound];
 Lower_Bound =[-T1bound T2bound_neg T3bound_neg -T4bound -T5bound -T6bound];
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% P =[-10,-11];
% F = place(A,B,P);
F =[-1200 -5000]; % Les poles de notre systeme avec retour d'états
syms T1 T2 T3 T4 T5 T6; % Les Angles THETA
T =[T1 T2 T3 T4 T5 T6]; % Le vecteur Angles


%[+90 +90 0 180 180 0]';2.44
X1 = Tin;  % Theta au cours du temps (Representation D'états)
%%%%%%%% Verification de l'existence du robot dans la marge des angles limites.%%%%%%%%%%%%%%%%%%%%


if (X1(1)>Upper_Bound(1) || X1(2)>Upper_Bound(2) || X1(3)>Upper_Bound(3) || X1(4)>Upper_Bound(4) || X1(5)>Upper_Bound(5) || X1(6)>Upper_Bound(6))
error('\nInitial Position out of range .\n\n Angles should be in Radians as follow \n\n T1 = +150°/-150°\n T2 = +40°/-107.5° \n T3 = +20°/-235° \n T4 = +350°/-350° \n T5 = +118°/-118° \n T6 = +350°/-350° ',class(F))
elseif(X1(1)<Lower_Bound(1) || X1(2)<Lower_Bound(2)|| X1(3)<Lower_Bound(3)|| X1(4)<Lower_Bound(4)|| X1(5)<Lower_Bound(5)|| X1(6)<Lower_Bound(6))
error('Initial Position out of range .  \n\n T1 = +150°/-150°\n T2 = +40°/-107.5° \n T3 = +20°/-235° \n T4 = +350°/-350° \n T5 = +118°/-118° \n T6 = +350°/-350° ',class(F))
end

%%%%%%%%%% La matrice de parametrage Denavit-Hartenberg %%%%%%%%%% %%%%%%%%%%%%%%%

DH = [+600 +pi/2 1100 T(1)+X1(1);...
    +1400 0 0 T(2)+X1(2);...
    +65 +pi/2 0 T(3)+X1(3);...
    0 +pi/2 +650+550 T(4)+X1(4);...
    0 +pi/2 0 T(5)+X1(5);...
    0 0 +372 T(6)+X1(6)];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%% Le calcul de la matrice de transformation finale %%%%%%%%%%%%%%%%%%

Tr = eye(4); % La matrice de transformation (initialisation à l'identité - Car ensuite on va multiplier )
Dr = [];     % Pour extraire les positions de chaque fin de lien (Utiliser pour dessiner le robot)
for i = 1 : length(T)
    L = [cos(DH(i,4)) -cos(DH(i,2))*sin(DH(i,4)) sin(DH(i,2))*sin(DH(i,4)) DH(i,1)*cos(DH(i,4));...
        sin(DH(i,4)) cos(DH(i,2))*cos(DH(i,4)) -sin(DH(i,2))*cos(DH(i,4)) DH(i,1)*sin(DH(i,4));...
        0 sin(DH(i,2)) cos(DH(i,2)) DH(i,3);...
        0 0 0 1];   % Substitution des parametres DH dans la matrice de transformation pour chaque joint (Boucle)
    %Dr = [Dr L(1:3,4)] 
    Tr = Tr * L; % La matrice de transformation en Symbolic
    Dr = [Dr Tr(1:3,4)]; % les postions de chaque fin de lien (Pour dessiner le robot)
end

%%%%%%%%%%% La position de chaque fin de lien en Symbolic
finale = [0;0;0];
for i = 1:6
 finale = [finale +Dr(:,i)];
end
finalenum = finale; % cette variable va etre utiliser par la suite pour convertir les Symbolic vers Numeric 
Trnum = Tr;         % cette variable va etre utiliser par la suite pour convertir les Symbolic vers Numeric
Ts = 10000;         % Period d'echantillonage (utiliser pour construire la commande du systeme - Indisponsable pour le traitement numerique de l'information analogique)
Trajectory = [];    % Variable utiliser pour dessiner la trajectoire parcourue par le 'End Effector' du robot.


%Tin = [3*1/Ts*pi +pi/2*1/Ts pi/2/Ts*1 0*-pi*1/Ts pi/2*1/Ts pi*1/Ts];
%Tin =[0 0 0 0 0 0];
%Tf  = [3*100/Ts*pi +pi/2*100/Ts pi/2/Ts*100 0*-pi*100/Ts pi/2*100/Ts pi*100/Ts];

%Tnum = [3*100/Ts*pi +pi/2*100/Ts pi/2/Ts*100 0*-pi*100/Ts pi/2*100/Ts pi*100/Ts];


Tinder1 = 0;Tinder2 = 0;Tinder3 = 0;Tinder4 = 0;Tinder5 = 0;Tinder6 = 0; % Les vitesses angulaires initiales
X2 = [Tinder1 Tinder2  Tinder3 Tinder4 Tinder5 Tinder6];

Tepsilon =[0.0010 0.001 0.003 0.001 0.005 0.005]; %La tolerance entre les angles actuels et les angles désirés
test = [];


%%%%%%%%%%%%%%%%%%%%% La Commande du systeme %%%%%%%%%%%%%%%%%%%%%%%%%%

%%% La Modélisation des systemes (Pendules soumis seulement aux moments des moteurs)
%%% X1' = X2
%%% X2' = U
%%% Y   = X1

%%% U =K(X-X0)
%%% La commande est un retour d'états  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

it = 0; % Nombre d'itirations
while ((abs(X1(1)-Tf(1))>Tepsilon(1) )|| (abs(X2(1))>Tepsilon(1))...
        ||(abs(X1(2)-Tf(2))>Tepsilon(2) )|| (abs(X2(2))>Tepsilon(2))...
        ||(abs(X1(3)-Tf(3))>Tepsilon(3) )|| (abs(X2(3))>Tepsilon(3))...
        ||(abs(X1(4)-Tf(4))>Tepsilon(4) )|| (abs(X2(4))>Tepsilon(4))...
        ||(abs(X1(5)-Tf(5))>Tepsilon(5) )|| (abs(X2(5))>Tepsilon(5))...
        ||(abs(X1(6)-Tf(6))>Tepsilon(6) )|| (abs(X2(6))>Tepsilon(6)))  % Conditions D'arret
%pause
grid on
    if (it == 50) %% Une Autre Condition d'arret au cas de divergence
        error('it = 50')
    end
    it = it+1;
    

%La modélisation et la commande des angles :

if (X1(1)<T1bound && X1(1)>-T1bound) % Verification que l'angles est toujours dans le bon intervalle   
U1 = (F(1)*(X1(1)-Tf(1))+F(2)*X2(1))/Ts; % Le calcul de la commande pour le premier angle%10000
X2(1) = X2(1) + U1;                      % L'integral de la vitesse angulaire(X2)
X1(1) = X1(1)+X2(1);                     % L'integral De l'angle (X1)

%%% Si l'angles atteint une limite, elle ne peut pas la dépasser
elseif (X1(1)<0)
    X1(1) = -T1bound;
    disp('lower bound 1')  %% Un message qui nous indique qu'elle limite est atteinte
elseif (X1(1)>0)
    X1(1) = T1bound;
    disp('upper bound 1')
end

%%%% Les memes étapes pour le reste des angles %%%

if (X1(2)<T2bound_pos && X1(2)>T2bound_neg)
U2 = --(F(1)*(X1(2)-Tf(2))+F(2)*X2(2))/Ts;
X2(2) = X2(2) +U2;
X1(2) = X1(2)+X2(2);
elseif (X1(2)<0)
    X1(2) = T2bound_neg;
    disp('lower bound 2')
elseif (X1(2)>0)
    X1(2) = T2bound_pos;
    disp('upper bound 2')
end
if (X1(3)<T3bound_pos && X1(3)>T3bound_neg)
U3 = --(F(1)*(X1(3)-Tf(3))+F(2)*X2(3))/Ts;
X2(3) = X2(3) +U3;
X1(3) = X1(3)+X2(3);
elseif (X1(3)<0)
    X1(3) = T3bound_neg;
    disp('lower bound 3')
elseif (X1(3)>0)
    X1(3) = T3bound_pos;
    disp('upper bound 3')
end
if (X1(4)<+T4bound && X1(4)>-T4bound)
U4 = --(F(1)*(X1(4)-Tf(4))+F(2)*X2(4))/Ts;
X2(4) = X2(4) +U4;
X1(4) = X1(4)+X2(4);
elseif (X1(4)<0)
    X1(4) = -T4bound;
    disp('lower bound 4')
elseif (X1(4)>0)
    X1(4) = +T4bound;
    disp('upper bound 4')
end
if (X1(5)<+T5bound && X1(5)>-T5bound)
U5 = --(F(1)*(X1(5)-Tf(5))+F(2)*X2(5))/Ts;
X2(5) = X2(5) +U5;
X1(5) = X1(5)+X2(5);
elseif (X1(5)<0)
    X1(5) = -T5bound;
    disp('lower bound 5')
elseif (X1(5)>0)
    X1(5) = +T5bound;
    disp('upper bound 5')
end
if (X1(6)<+T6bound && X1(6)>-T6bound)
U6 = --(F(1)*(X1(6)-Tf(6))+F(2)*X2(6))/Ts;
X2(6) = X2(6) +U6;
X1(6) = X1(6)+X2(6);
elseif (X1(6)<0)
    X1(6) = -T6bound;
    disp('lower bound 6')
elseif (X1(6)>0)
    X1(6) = +T6bound;
    disp('upper bound 6')
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%hold on 

%X1 = [X1(1),X1(2),X1(3),X1(4),X1(5),X1(6)] ;
%Theeta = X1(1);
% test = [test U1];
% plot(test)%pause
% drawnow

%%% Cette partie fait la conversion des syms vers num dans la matrice de
%%% transformation et aussi le dessin du robot
finalenum = finale;
Trnum = Tr;
for i =1 : 6
   Trnum =  subs(Trnum,T(i),X1(i));
   finalenum =  subs(finalenum,T(i),X1(i));
end


%Trnum = double(Trnum)
%plot3(finalenum(1,:),finalenum(2,:),finalenum(3,:),'linewidth',4)

grid on
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
    if (it == 1) %% Une Autre Condition d'arret au cas de divergence
         sgtitle('Appuyer sur un button du clavier pour lancer la simulation')%suptitle('Appuyer sur un button du clavier pour lancer la simulation')
        pause
    end
drawnow update

hold off

Trajectory = [Trajectory [finalenum(1,7), finalenum(2,7),finalenum(3,7)]'];
%pause
 plot3(Trajectory(1,1:it-1),Trajectory(2,1:it-1),Trajectory(3,1:it-1))
hold on
box on
ax = gca;
ax.ZGrid = 'on';
ax.XGrid = 'on';
ax.YGrid = 'on';
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
%  hold on 
%  plot3(Trajectory(1,:),Trajectory(2,:),Trajectory(3,:),'Linewidth',3)

