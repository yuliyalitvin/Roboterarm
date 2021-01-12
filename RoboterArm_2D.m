
% Armlängen angeben
    armglied_L1 = 0.2; 
    armglied_L2 = 0.2;
    armglied_L3 = 0.2;

 roboterarm = roboterArm([armglied_L1 armglied_L2 armglied_L3], 0.6)
    
% Der zu umlaufende Pfad wird generiert. In diesem Fall ein Kreis in der
% xy-Ebene und mit einem Radius von 0.15
    t = (0:1:9)'; % Zeitschritt
    zaehler = length(t);
%     mittelpunkt = [0.3 0.1 0];
%     radius = 0.15;
%     theta = t*(2*pi/t(end));
%     punkte = mittelpunkt + radius*[cos(theta) sin(theta) zeros(size(theta))];

    % Eigenschaften der einzelnen Ablegemarkierungen
    am1_x = -0.6;   % x Position
    am1_y = 0.5;    % y Position
    am1_b = 0.25;   % b Breite
    am1_h = 0.1;    % h Hoehe
    
    am2_x = -0.125; 
    am2_y = 0.5; 
    am2_b = 0.25; 
    am2_h = 0.1;
    
    am3_x = 0.35; 
    am3_y = 0.5; 
    am3_b = 0.25; 
    am3_h = 0.1;
    
    % Eigenschaften der einzelnen Kisten 
    k1_x = -0.25;   
    k1_y = -0.1;      
    k1_b = 0.07;    
    k1_h = 0.07;    
    
    k2_x = 0; 
    k2_y = -0.5; 
    k2_b = 0.07; 
    k2_h = 0.07; 
    
    k3_x = 0.5; 
    k3_y = -0.25; 
    k3_b = 0.07; 
    k3_h = 0.07; 
    
    ursprungspunkt_endeffektor = [0 0 0]; 
    
    % Mittelpunkte der Ablegemarkierungen
    mittelpunkt_a_m_1 = [(am1_x + (am1_x + am1_b))/2 (am1_y + (am1_y + am1_h))/2 0];    % ablege_markierung_1 = rectangle('Position', [am1_x am1_y am1_b am1_h])
    mittelpunkt_a_m_2 = [(am2_x + (am2_x + am2_b))/2 (am2_y + (am2_y + am2_h))/2 0];         % ablege_markierung_2 = rectangle('Position', [am2_x am2_y am2_b am2_h])
    mittelpunkt_a_m_3 = [(am3_x + (am3_x + am3_b))/2 (am3_y + (am3_y + am3_h))/2 0];     % ablege_markierung_3 = rectangle('Position', [am3_x am3_y am3_b am3_h])
    
    % Mittelpunkte der Kisten
    mittelpunkt_k_1 = [(k1_x + (k1_x + k1_b))/2 (k1_y + (k1_y + k1_h))/2 0];    % kiste_1 = rectangle('Position', [k1_x k1_y k1_b k1_h])
    mittelpunkt_k_2 = [(k2_x + (k2_x + k1_b))/2 (k2_y + (k2_y + k2_h))/2 0];     % kiste_2 = rectangle('Position', [k2_x k2_y k2_b k2_h])
    mittelpunkt_k_3 = [(k3_x + (k3_x + k3_b))/2 (k3_y + (k3_y + k3_h))/2 0];     % kiste_3 = rectangle('Position', [k3_x k3_y k3_b k3_h])
    
    % Punkte, die der Roboterarm ansteuert
    punkte = [ursprungspunkt_endeffektor; mittelpunkt_k_1; mittelpunkt_a_m_1; ursprungspunkt_endeffektor; mittelpunkt_k_2; mittelpunkt_a_m_2; ursprungspunkt_endeffektor; mittelpunkt_k_3; mittelpunkt_a_m_3; ursprungspunkt_endeffektor] 

% Ein Objekt der Klasse "inverseKinematics" erzeugen, das dem Roboterarm
% einen Loesung zur Pfadfindung findet, damit der Endeffektor sich an
% diesem Pfad entlang bewegen kann.
% Konfigurationsloesungen (Pfadfindung) als Matrix matrix_konfigurationsloesungen vorverteilen
    ursprungs_konfiguration = homeConfiguration(roboterarm);
    ndof = length(ursprungs_konfiguration); % To-Do: herausfinden was "ndof" heißen/sein soll
    matrix_konfigurationsloesungen = zeros(zaehler, ndof); 

% Kreieren des Berechners fuer die inverse Kinematik
    inverse_kinematik = inverseKinematics('RigidBodyTree', roboterarm);
    gewichtung = [0, 0, 0, 1, 1, 0]; % weights
    endeffektor = 'endE_tool';

% In einem Loop durch die Pfadpunkte durchgehen, um den Kreis nachzufahren.
% Das Objekt inverse_kinematik fuer jeden PUnkt erneut aufrufen, um die
% Gelenkkonfiguration zu erzeugen, mit der die Endeffektorposition erreicht
% wird.
    initiale_konfiguration = ursprungs_konfiguration; % Initialisierung mithilfe der Ursprungskonfiguration
    for i = 1:zaehler
        % Loesung fuer die Konfiguration, die die gewuenschte
        % Endeffektorposition erfuellt
        punkt = punkte(i,:);
        konfigurations_loesung = inverse_kinematik(endeffektor, trvec2tform(punkt), gewichtung, initiale_konfiguration); % qSol
        % Konfiguration speichern 
        matrix_konfigurationsloesungen(i,:) = konfigurations_loesung;
        % mit vorheriger Loesung starten 
        initiale_konfiguration = konfigurations_loesung;
    end

% Loesung animieren
    figure
    show(roboterarm,matrix_konfigurationsloesungen(1,:)'); % Roboter fuer jeden Rahmen der Lesung unter Verwendung spezifischer Roboterkonfiguration plotten 
%     view(2) % Perspektive --> frontview
    view(3) % Perspektive --> sideview
    ax = gca; % gibt die aktuellen Achsen fuer die aktuelle Figure zurueck
    ax.Projection = 'orthographic'; % die Art wie man die Achsen anzeigen laesst --> Perspektive, hier: senkrecht
    hold on
    %plot(punkte(:,1), punkte(:,2), 'k') % Kreis bzw. den zu fahrenden Pfad plotten 
    hold on 
    
    % Tisch - grau
    boden = rectangle('Position',[-0.75 -0.75 1.5 1.5])
    boden.FaceColor = [0.9 0.9 0.9];
    hold on 
    
    % Ablegemarkierung 1 - rot 
    ablege_markierung_1 = rectangle('Position', [am1_x am1_y am1_b am1_h])
    ablege_markierung_1.FaceColor = 'red'; 
    hold on
    
    % Ablegemarkierung 2 - gelb
    ablege_markierung_2 = rectangle('Position', [am2_x am2_y am2_b am2_h])
    ablege_markierung_2.FaceColor = 'yellow';
    hold on
    
    % Ablegemarkierung 3 - gruen
    ablege_markierung_3 = rectangle('Position', [am3_x am3_y am3_b am3_h])
    ablege_markierung_3.FaceColor = 'green';
    hold on
   
    
    % Kiste 1 - rot
    kiste_1 = rectangle('Position', [k1_x k1_y k1_b k1_h])
    kiste_1.FaceColor = 'red'; 
    %kiste_1.EdgeColor = 'red'; --> waere auch eine Alternative
    hold on 
   
    % Kiste 2 - gelb
    kiste_2 = rectangle('Position', [k2_x k2_y k2_b k2_h])
    kiste_2.FaceColor = 'yellow';
    hold on
    
    % Kiste 3 - gruen
    kiste_3 = rectangle('Position', [k3_x k3_y k3_b k3_h])
    kiste_3.FaceColor = 'green'; 
    axis([-1 1 -1 1])

% Schnelligkeit des Pfadabfahrens vom Roboterarm einstellen
    frames_pro_sekunde = 1;
    rate_kontrolle = rateControl(frames_pro_sekunde);
    for i = 1:zaehler
        show(roboterarm, matrix_konfigurationsloesungen(i,:)', 'PreservePlot', false); % Roboterarm in jedem Durchlauf auf der aktuellen Position angezeigt
        if(i == 3)
            % translate(kiste_1, [-0.5, -0.3, 0])
            set(kiste_1, 'Position', [(am1_x + (am1_b/3)) am1_y k1_b k1_h])
        elseif(i == 6)
            set(kiste_2, 'Position', [(am2_x + (am2_b/3)) am2_y k2_b k2_h])
        elseif(i == 9)
            set(kiste_3, 'Position', [(am3_x + (am3_b/3)) am3_y k2_b k2_h])
        end 
        drawnow
        waitfor(rate_kontrolle); % Pausieren der Ausfuehrung bis der Code die gewuenschte Ausfuehrungsrate erreicht
        
%           filename = '2D_Roboterarm_frontview.gif';
%           frame = getframe(gcf);
%           [SIf,cm] = rgb2ind(frame.cdata,256);
%           if i == 1
%             imwrite(SIf,cm,filename,'Loop',Inf,'Delay',1);
%           else
%             imwrite(SIf,cm, filename,'WriteMode','append','Delay',1);
%           end
          
          filename = '2D_Roboterarm_sideview.gif';
          frame = getframe(gcf);
          [SIf,cm] = rgb2ind(frame.cdata,256);
          if i == 1
            imwrite(SIf,cm,filename,'Loop',Inf,'Delay',1);
          else
            imwrite(SIf,cm, filename,'WriteMode','append','Delay',1);
          end
    end

