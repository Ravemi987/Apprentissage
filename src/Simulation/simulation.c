#include "simulation.h"

/* ========= PHYSIQUE ========= */

static double wrapAngle(double angle) {
    angle = fmod(angle + MATH_PI, 2.0 * MATH_PI);
    if (angle < 0.0) angle += 2.0 * MATH_PI; 
    return angle - MATH_PI;
}


Drone createDrone(double x, double y, double z) {
    Drone d;

    // Met tout à 0
    memset(&d, 0, sizeof(Drone));
    d.x = x; d.y = y; d.z = z;
    d.battery_remaining = 0;
    
    // Initialisation des cibles (Hover par défaut)
    d.target_thrust = M * G; 
    d.target_roll = 0.0;
    d.target_pitch = 0.0;
    d.target_yaw = 0.0;

    // Initialisation des PID
    d.pid_roll  = (PIDController){ .kp = 1.5, .ki = 0.0, .kd = 0.8, .integral = 0, .prev_error = 0 };
    d.pid_pitch = (PIDController){ .kp = 1.5, .ki = 0.0, .kd = 0.8, .integral = 0, .prev_error = 0 };
    d.pid_yaw   = (PIDController){ .kp = 1.0, .ki = 0.0, .kd = 0.2, .integral = 0, .prev_error = 0 };
    
    return d;
}


/* Fonction pour détecter les collisions avec les utilisateurs, similaire à collisionWithObstacle */
int collisionWithUser(World *w) {
    Drone *d = w->drone;

    for (int i = 0; i < w->numUsers; i++) {
        User u = w->users[i];
        
        // Un piéton est au sol (z=0) et mesure environ 2m de haut
        if (d->z >= 0.0 && d->z <= 2.0) {
            double dist_horiz = sqrt(pow(d->x - u.x, 2) + pow(d->y - u.y, 2));
            if (dist_horiz <= 1.0) return 1;
        }
    }
    return 0;
}


int collisionWithObstacle(World *w) {
    Drone *d = w->drone;

    for (int i = 0; i < w->numObstacles; i++) {
        Obstacle3D obs = w->obstacles[i];
        // On vérifie si le drone est en dessous de la hauteur de l'obstacle (mais pas forcément dedans)
        if (d->z >= obs.z && d->z <= (obs.z + obs.height)) {
            // On calcul la distance horizontale pour détecter si le drone est réellement dedans
            double dist_horiz = sqrt(pow(d->x - obs.x, 2) + pow(d->y - obs.y, 2));
            if (dist_horiz <= obs.radius) return 1;
        }
    }

    return 0;
}


/* Similaire à applyLimits, mais renvoie seulement si oui ou non le drone atteint une limite */
int isDroneCrashed(World *w) {
    Drone *d = w->drone;

    if(d->x <=0 || d->x >= w->width) return 1;
    if(d->y <=0 || d->y >= w->height) return 1;
    if(d->z <=0 || d->z >= w->depth) return 1;

    if (collisionWithObstacle(w)) return 1;

    if (collisionWithUser(w)) return 1;

    return 0;
}


void handleCollisionWithObstacle(World * w) {
    Drone *d = w->drone;

    for (int i = 0; i < w->numObstacles; i++) {
        Obstacle3D obs = w->obstacles[i];
        
        // Si le drone est dans la tranche verticale de l'obstacle
        if (d->z >= obs.z && d->z <= (obs.z + obs.height)) {
            double dist_horiz = sqrt(pow(d->x - obs.x, 2) + pow(d->y - obs.y, 2));

            // S'il a pénétré à l'intérieur du rayon de l'obstacle
            if (dist_horiz <= obs.radius) {

                // Annulation des viteses (L'impact arrête le mouvement)
                d->x_dot = 0.0;
                d->y_dot = 0.0;
                d->z_dot = 0.0;
            }
        }
    }
}


void handleCollisionWithUsers(World *w) {
    Drone *d = w->drone;

    for (int i = 0; i < w->numUsers; i++) {
        User u = w->users[i];
        
        // Un piéton est au sol (z=0) et mesure environ 2m de haut
        if (d->z >= 0.0 && d->z <= 2.0) {
            double dist_horiz = sqrt(pow(d->x - u.x, 2) + pow(d->y - u.y, 2));
            double user_radius = 1.0;

            // S'il rentre en collision physique avec l'utilisateur
            if (dist_horiz <= user_radius) {

                // Annulation des vitesses
                d->x_dot = 0.0;
                d->y_dot = 0.0;
                d->z_dot = 0.0;
            }
        }
    }
}


/* Ajoute des contraintes au déplacement physiques quand on le pilote (limites de la carte) */
void applyLimits(World *w) {
    Drone *d = w->drone;

    // Limites de la carte

    // Axe X
    if (d->x < 0) { d->x = 0; d->x_dot = 0; }
    if (d->x > w->width) { d->x = w->width; d->x_dot = 0; }

    // Axe Y
    if (d->y < 0) { d->y = 0; d->y_dot = 0; }
    if (d->y > w->height) { d->y = w->height; d->y_dot = 0; }

    // Axe Z 
    if (d->z > w->depth) { d->z = w->depth; d->z_dot = 0; }

    // Collision avec le sol (Z vers le haut)
    if (d->z <= 0) {
        d->z = 0;
        d->z_dot = 0;
        
        d->x_dot *= 0.8;
        d->y_dot *= 0.8;
        
        d->p = 0; d->q = 0; d->r = 0;
        d->phi = 0; d->theta = 0;
    }

    // Collision avec les obstacles (interdiction de traverser)
    handleCollisionWithObstacle(w);

    // Collision avec les utilisateurs
    handleCollisionWithUsers(w);
}


/* Traduit les ordres en vitesse moteur */
void handleCommand(Drone *d, int action) {
    d->target_roll = 0.0;
    d->target_pitch = 0.0;
    d->target_thrust = M * G;

    // Mode par défaut (autonome)
    double thrust_boost = 4.0;
    double yaw_increment = 0.05;
    double current_angle_limit = ANGLE_LIMIT;

    if (d->is_autonomous_mode == 0) {
        thrust_boost = 12.0;
        yaw_increment = 0.02;
        current_angle_limit = 0.50;
    }

    if (action == ENGINE_UP)            d->target_thrust = (M * G) + thrust_boost; // Monter
    if (action == ENGINE_DOWN)          d->target_thrust = (M * G) - thrust_boost; // Descendre
    if (action == ENGINE_PITCH_LEFT)    d->target_pitch = current_angle_limit;   // Pitch avant
    if (action == ENGINE_PITCH_RIGHT)   d->target_pitch = -current_angle_limit;  // Pitch arrière
    if (action == ENGINE_ROLL_LEFT)     d->target_roll = -current_angle_limit;   // Roll gauche
    if (action == ENGINE_ROLL_RIGHT)    d->target_roll = current_angle_limit;    // Roll droite

    if (action == ENGINE_YAW_LEFT) {
        d->target_yaw -= yaw_increment;
        d->target_yaw = wrapAngle(d->target_yaw); // Corrige le bug de la toupie
    }
    if (action == ENGINE_YAW_RIGHT) {
        d->target_yaw += yaw_increment;
        d->target_yaw = wrapAngle(d->target_yaw); // Corrige le bug de la toupie
    }
}


double updatePID(PIDController *pid, double target, double current, double dt) {
    double error = target - current;
    
    // Proportional
    double p_out = pid->kp * error;
    
    // Integral (avec une limite anti-windup conseillée)
    pid->integral += error * dt;
    double i_out = pid->ki * pid->integral;
    
    // Derivative (basée sur le changement de la mesure pour éviter le kick)
    double derivative = (current - pid->prev_error) / dt;
    double d_out = - pid->kd * derivative;
    
    pid->prev_error = current;
    
    return p_out + i_out + d_out;
}


void applyControllerAndMixer(Drone *d, double dt) {
    double U[4];
    
    // U[0] est la poussée brute désirée
    U[0] = d->target_thrust; 
    
    // U[1], U[2], U[3] sont générés par le PID pour corriger les erreurs angulaires
    U[1] = updatePID(&d->pid_roll, d->target_roll, d->phi, dt);
    U[2] = updatePID(&d->pid_pitch, d->target_pitch, d->theta, dt);
    U[3] = updatePID(&d->pid_yaw, d->target_yaw, d->psi, dt);

    // On transforme la commande virtuelle U en vitesse de rotation des 4 moteurs (w2)
    double w2[4];
    w2[0] = U[0]/(4*B) + U[2]/(2*B) - U[3]/(4*D); // Moteur Avant
    w2[1] = U[0]/(4*B) - U[1]/(2*B) + U[3]/(4*D); // Moteur Gauche
    w2[2] = U[0]/(4*B) - U[2]/(2*B) - U[3]/(4*D); // Moteur Arrière
    w2[3] = U[0]/(4*B) + U[1]/(2*B) + U[3]/(4*D); // Moteur Droite

    // Enregistre les vitesses dans le drone
    for(int i=0; i<4; i++) {
        d->omega[i] = sqrt(fmax(0, w2[i])); 
    }
}


/* La physique ne connaît pas les commandes directement, c'est pour cela que nous recalculons les U réels */
void computeCommandVector(World *w, double *U) {
    Drone d = *(w->drone);

    U[0] = B * (pow(d.omega[0], 2) + pow(d.omega[1], 2) + pow(d.omega[2], 2) + pow(d.omega[3], 2)); // Poussee totale
    U[1] = L * B * (- pow(d.omega[1], 2) + pow(d.omega[3], 2)); // Moment de Roll
    U[2] = L * B * (pow(d.omega[0], 2) - pow(d.omega[2], 2)); // Moment de Pitch
    U[3] = D * (- pow(d.omega[0], 2) + pow(d.omega[1], 2) - pow(d.omega[2], 2) + pow(d.omega[3], 2)); // Moment de Yaw
}


void computeAngularAccelerations(World *w, double *U) {
    Drone *d = w->drone;
    double damping = 0.1;
    
    // Dérivées des angles à partir des vitesses angulaires
    d->phi_dot = d->p + d->q * sin(d->phi) * tan(d->theta) + d->r * cos(d->phi) * tan(d->theta);
    d->theta_dot = d->q * cos(d->phi) - d->r * sin(d->phi);
    d->psi_dot = d->q * (sin(d->phi) / cos(d->theta)) + d->r * (cos(d->phi) / cos(d->theta));

    // Accélérations angulaires à partir des moments
    d->p_dot = ( d->theta_dot * d->psi_dot * (Iyy - Izz) + U[1] ) / Ixx - (damping * d->p);
    d->q_dot = ( d->phi_dot * d->psi_dot * (Izz - Ixx) + U[2] ) / Iyy - (damping * d->q);
    d->r_dot = ( d->theta_dot * d->phi_dot * (Ixx - Iyy) + U[3] ) / Izz - (damping * d->r);
}


void updateAngularVelocities(World *w, double dt) {
    Drone *d = w->drone;

    // Mise à jour des vitesses angulaires
    d->p = d->p + (d->p_dot * dt);
    d->q = d->q + (d->q_dot * dt);
    d->r = d->r + (d->r_dot * dt);

    double true_rot = (d->is_autonomous_mode) ? MAX_ROT : 5.0;

    // Sécurité : plafonner la vitesse de rotation
    if (d->p > true_rot) d->p = true_rot; else if (d->p < -true_rot) d->p = -true_rot;
    if (d->q > true_rot) d->q = true_rot; else if (d->q < -true_rot) d->q = -true_rot;
    if (d->r > true_rot) d->r = true_rot; else if (d->r < -true_rot) d->r = -true_rot;
}


void updateVelocities(World *w, double dt) {
    Drone *d = w->drone;

    // Mise à jour des vitesses linéaires
    d->x_dot = d->x_dot + (d->x_dot_dot * dt);
    d->y_dot = d->y_dot + (d->y_dot_dot * dt);
    d->z_dot = d->z_dot + (d->z_dot_dot * dt);

    double true_speed = (d->is_autonomous_mode) ? MAX_VELOCITY : 30.0;

    if (d->x_dot > true_speed)  d->x_dot = true_speed;
    if (d->x_dot < -true_speed) d->x_dot = -true_speed;
    if (d->y_dot > true_speed)  d->y_dot = true_speed;
    if (d->y_dot < -true_speed) d->y_dot = -true_speed;
    if (d->z_dot > true_speed)  d->z_dot = true_speed;
    if (d->z_dot < -true_speed) d->z_dot = -true_speed;
}


void updateOrientation(World *w, double dt) {
    Drone *d = w->drone;

    // Mise à jour des angles d'orientation
    d->phi = d->phi + (d->phi_dot * dt);
    d->theta = d->theta + (d->theta_dot * dt);
    d->psi = d->psi + (d->psi_dot * dt);

    // Empeche l'explosion des gradients
    d->phi = wrapAngle(d->phi);
    d->psi = wrapAngle(d->psi);
    d->theta = wrapAngle(d->theta);

    double current_angle_limit = (d->is_autonomous_mode) ? ANGLE_LIMIT : 0.50;

    if (d->theta > current_angle_limit)  d->theta = current_angle_limit;
    if (d->theta < -current_angle_limit) d->theta = -current_angle_limit;
}


void computeAccelerations(World *w, double *U) {
    Drone *d = w->drone;
    double thrustFactor = U[0] / M;

    // Accélérations incluant la friction. On soustrait (Coeff * Vitesse) à l'accélération
    d->z_dot_dot = (cos(d->phi) * cos(d->theta)) * thrustFactor - G - (DRAG_COEFF * d->z_dot / M);
    d->x_dot_dot = (sin(d->phi) * sin(d->psi) + cos(d->psi) * sin(d->theta) * cos(d->phi)) * thrustFactor - (DRAG_COEFF * d->x_dot / M);
    d->y_dot_dot = (- sin(d->phi) * cos(d->psi) + sin(d->psi) * sin(d->theta) * cos(d->phi)) * thrustFactor - (DRAG_COEFF * d->y_dot / M);
}


void updatePosition(World *w, double dt) {
    Drone *d = w->drone;

    // Mise à jour de la position
    d->x = d->x + (d->x_dot * dt);
    d->y = d->y + (d->y_dot * dt);
    d->z = d->z + (d->z_dot * dt);
}



/* Mise à jour des caractéristiques du drone */
void physicsStep(World *w, double dt) {
    double U[4];
    
    applyControllerAndMixer(w->drone, dt); // Le PID lit la position actuelle et ajuste la vitesse des moteurs (omega)
    computeCommandVector(w, U); // La physique calcule les vrais efforts U à partir des moteurs (omega)

    computeAngularAccelerations(w, U);
    updateAngularVelocities(w, dt);
    updateOrientation(w, dt);
    computeAccelerations(w, U);
    updateVelocities(w, dt);
    updatePosition(w, dt);
    applyLimits(w);
}


/* ========= RESEAU ========= */


/* Log-Distance Path Loss Model :
 * La puissance du signal WiFi décroît en fonction de la distance

 * RSSI = Ptx ​− 10 * n * log10​(d)
 * où Ptx est la puissance d'émission, n le coefficient de perte et d la distance
*/
double computeRSSI(Drone *d, User *u) {
    double dist = sqrt(pow(d->x - u->x, 2) + pow(d->y - u->y, 2) + pow(d->z - u->z, 2)) + 0.001;  // On évite log10(0)
    if (dist < 1.0) dist = 1.0; // On met une limite de distance, car le 0.001 ne permet pas d'éviter le log(0) en pratique
    return SIGNAL_BASE_POWER - (10 * PATH_LOSS_EXPONENT * log10(dist));
}


/* ========= SAVE ========= */


/* Exporte l'état du monde dans un fichier JSON pour l'interface Web */
void exportStateToJSON(World *w, const char *filepath) {
    char temp_path[256];
    snprintf(temp_path, sizeof(temp_path), "%s.tmp", filepath);
    FILE *f = fopen(temp_path, "w");
    if (f == NULL) return;

    fprintf(f, "{\n");
    fprintf(f, "  \"drone\": {\n");
    fprintf(f, "    \"x\": %.2f, \"y\": %.2f, \"z\": %.2f,\n", w->drone->x, w->drone->y, w->drone->z);
    fprintf(f, "    \"phi\": %.4f, \"theta\": %.4f, \"psi\": %.4f,\n", w->drone->phi, w->drone->theta, w->drone->psi);
    fprintf(f, "    \"vx\": %.2f, \"vy\": %.2f, \"vz\": %.2f\n", w->drone->x_dot, w->drone->y_dot, w->drone->z_dot);
    fprintf(f, "  },\n");
    
    // Utilisateurs
    fprintf(f, "  \"users\": [\n");
    for (int i = 0; i < w->numUsers; i++) {
        fprintf(f, "    {\"x\": %.2f, \"y\": %.2f, \"z\": %.2f}", w->users[i].x, w->users[i].y, w->users[i].z);
        if (i < w->numUsers - 1) fprintf(f, ",\n");
    }
    fprintf(f, "  ],\n"); // Ajout de la virgule ici !

    // Obstacles
    fprintf(f, "  \"obstacles\": [\n");
    for (int i = 0; i < w->numObstacles; i++) {
        fprintf(f, "    {\"x\": %.2f, \"y\": %.2f, \"z\": %.2f, \"radius\": %.2f, \"height\": %.2f}", 
                w->obstacles[i].x, w->obstacles[i].y, w->obstacles[i].z, w->obstacles[i].radius, w->obstacles[i].height);
        if (i < w->numObstacles - 1) fprintf(f, ",\n");
    }
    fprintf(f, "  ]\n");

    fprintf(f, "}\n");
    fclose(f);

    rename(temp_path, filepath);
}
