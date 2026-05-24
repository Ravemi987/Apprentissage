#include "env.h"
#include <generation.h>
#include <stdlib.h>

/* Fonction utilitaire pour limiter une valeur entre un min et un max (Hard
 * Clip) */
double clamp(double val, double min_val, double max_val) {
  if (isnan(val) || isinf(val))
    return 0.0;

  if (val < min_val)
    return min_val;
  if (val > max_val)
    return max_val;
  return val;
}

// Paramètres de la récompense batterie.
// alpha = fraction de max_steps conservée comme marge de sécurité.
// Plus alpha est grand, plus le drone part en retour tôt.
// Plus alpha est petit, plus l'agent explore avant de rentrer.
#define BATTERY_RETURN_MARGIN_ALPHA 0.15
#define BATTERY_RETURN_BASE_RADIUS 2.5
#define BATTERY_RETURN_SCALE 50.0
#define BATTERY_RETURN_BASE_BONUS 0.25
#define BATTERY_RETURN_WARNING_WEIGHT 0.35
#define BATTERY_RETURN_EMERGENCY_WEIGHT 1.0

static int isAtBase(Env *env) {
  Drone *d = env->physical_world->drone;
  double dx = d->x - env->spawn_drone_x;
  double dy = d->y - env->spawn_drone_y;
  double dz = d->z - env->spawn_drone_z;

  return sqrt(dx * dx + dy * dy + dz * dz) <= BATTERY_RETURN_BASE_RADIUS;
}

/*
 * Récompense associée à la batterie.
 * Zone 1 : marge suffisante -> pas de pénalité.
 * Zone 2 : marge faible -> pénalité douce pour inciter à rentrer.
 * Zone 3 : retour trop tard / batterie critique -> pénalité forte.
 *
 * La marge est basée sur alpha * max_steps, donc alpha agit comme "réserve de
 * sécurité".
 * - alpha grand : retour anticipé, plus sûr, mais moins de temps pour optimiser
 * le signal.
 * - alpha petit : exploration plus agressive, mais risque plus élevé de finir
 * sans énergie.
 */
static double getBatteryReward(Env *env) {
  Drone *d = env->physical_world->drone;
  double home_distance = sqrt(pow(d->x - env->spawn_drone_x, 2.0) +
                              pow(d->y - env->spawn_drone_y, 2.0) +
                              pow(d->z - env->spawn_drone_z, 2.0));

  if (d->battery_remaining == 0) {
    if (isAtBase(env))
      return BATTERY_RETURN_BASE_BONUS;
    return -100.0;
  }

  // Temps estimé pour revenir à la base. On utilise une vitesse de retour
  // conservative.
  double v_ref = 2.0;
  int return_steps = (int)(home_distance / v_ref);
  if (home_distance > 0.0 && return_steps == 0)
    return_steps = 1;

  // Marge de sécurité : on démarre la pénalité avant d’être vraiment au bord.
  double margin_steps =
      fmax(30.0, BATTERY_RETURN_MARGIN_ALPHA * (double)env->max_steps);

  // Encore assez de batterie pour poursuivre l’exploitation du signal.
  if ((double)d->battery_remaining > (double)return_steps + margin_steps) {
    return 0.0;
  }

  // Zone "attention" : on commence à pousser le drone à revenir.
  if ((double)d->battery_remaining > (double)return_steps) {
    double urgency =
        (return_steps + margin_steps - (double)d->battery_remaining) /
        margin_steps;
    return -BATTERY_RETURN_WARNING_WEIGHT * urgency *
           (home_distance / BATTERY_RETURN_SCALE);
  }

  // Zone critique : le retour est urgent, et la pénalité augmente avec la
  // distance à la base.
  double urgency = (return_steps - (double)d->battery_remaining + 1.0) /
                   fmax(1.0, (double)return_steps);
  double distance_penalty = home_distance / BATTERY_RETURN_SCALE;
  return -BATTERY_RETURN_EMERGENCY_WEIGHT * urgency * (1.0 + distance_penalty);
}

/* Calcule la pénalité liée aux obstacles (Arbres, bâtiments) */
static double getObstaclePenalty(World *w, Drone *d) {
  double penalty = 0.0;
  for (int i = 0; i < w->numObstacles; i++) {
    Obstacle3D obs = w->obstacles[i];

    // Si le drone vole à la hauteur de l'obstacle
    if (d->z >= obs.z && d->z <= (obs.z + obs.height)) {
      double dist_h = sqrt(pow(d->x - obs.x, 2) + pow(d->y - obs.y, 2));
      double safety_zone = obs.radius + SAFETY_RADIUS;

      if (dist_h < safety_zone) {
        double penetration = (safety_zone - dist_h) / SAFETY_RADIUS;
        penalty -= clamp(penetration, 0.0, 1.0) * 1.0;
      }
    }
  }
  return penalty;
}

/* Calcule la pénalité de sécurité humaine (Ne pas voler trop bas au-dessus des
 * gens) */
static double getHumanProximityPenalty(World *w, Drone *d) {
  double penalty = 0.0;

  if (d->z >= 0.0 && d->z <= SAFETY_RADIUS) {
    for (int i = 0; i < w->numUsers; i++) {
      User u = w->users[i];
      double dist_h = sqrt(pow(d->x - u.x, 2) + pow(d->y - u.y, 2));

      if (dist_h < SAFETY_RADIUS) {
        double penetration = (SAFETY_RADIUS - dist_h) / SAFETY_RADIUS;
        penalty -= clamp(penetration, 0.0, 1.0) * 1.0;
      }
    }
  }
  return penalty;
}

/* Calcule la moyenne normalisée du signal pour tous les utilisateurs (0.0
 * à 1.0) */
static double getAverageSignalNorm(World *w, Drone *d,
                                   int *out_connected_count) {
  if (w->numUsers <= 0)
    return 0.0;

  double total_rssi_norm = 0.0;
  *out_connected_count = 0;

  for (int i = 0; i < w->numUsers; i++) {
    double rssi = computeRSSI(d, &w->users[i]);
    if (isnan(rssi) || isinf(rssi))
      rssi = -100.0;

    // Normalisation (Pire : -100dBm -> 0.0 | Parfait : -30dBm -> 1.0)
    double norm = clamp((rssi + 100.0) / 70.0, 0.0, 1.0);
    total_rssi_norm += norm;

    if (rssi > SIGNAL_BASE_POWER)
      (*out_connected_count)++;
  }

  return total_rssi_norm / w->numUsers;
}

/* Fonction Principale de Récompense */
double getReward(Env *env) {
  World *w = env->physical_world;
  Drone *d = w->drone;

  // Pénalité de temps (orce l'agent à être efficace)
  double reward = -0.05;

  // Gestion du Signal (Absolu + Delta)
  int connected = 0;

  // Attention, garder garder la valeur moyenne et surtout pasfaire une
  // différence !
  double current_signal_norm = getAverageSignalNorm(w, d, &connected);

  if (w->numUsers > 0) {
    // Si le signal est parfait (1.0), il gagne +1.0, ce qui annule la pénalité
    // de temps (-0.05) et encourage le hovering.
    reward += current_signal_norm * 1.0;

    if (connected == w->numUsers)
      reward += 0.05; // Bonus de réussite
  }

  // Application des pénalités environnementales
  reward += getObstaclePenalty(w, d);
  reward += getHumanProximityPenalty(w, d);

  // Pénalité linéaire globale
  double speed_squared = pow(d->x_dot, 2) + pow(d->y_dot, 2) + pow(d->z_dot, 2);
  reward -= speed_squared * 0.003;

  // Pénalité angulaire modérée (Pour diminuer le Yaw, très sensible avec un
  // impact invisible, sans interdire de tourner)
  double angular_speed = pow(d->p, 2) + pow(d->q, 2) + pow(d->r, 2);
  reward -= angular_speed * 0.001;

  return reward;
}

/*
 * Cette fonction définit de quelles informations l'IA a besoin pour savoir ce
 * que doit faire le drone à chaque instant.  Elle renvoit donc un vecteur de
 * valeurs qui définissent notre monde et qui set d'entrée au réseau de neurone.
 * Cela inclut les positions relatives aux utilisateurs, mais aussi aux
 * obstacles. Voir les commentaires et aussi le fichier env.h pour plus d'infos.
 */
void getStateVector(Env *env, double *state_out) {
  World *w = env->physical_world;
  Drone *d = w->drone;

  // Premiere étape : remplir la grille d'utilisateur
  fillUsersGrid(env, state_out);

  // Deuxième étape : capture des obstacles (LiDAR)
  captureObstacles(env, state_out);

  // Troisième étape : variables physiques du drone (comme avec une target A ->
  // B)
  int drone_offset = GRID_SIZE * GRID_SIZE + (MAX_CLOSEST_OBSTACLES * 3);

  state_out[drone_offset + 0] = clamp(d->x / w->width, 0.0, 1.0);
  state_out[drone_offset + 1] = clamp(d->y / w->height, 0.0, 1.0);
  state_out[drone_offset + 2] = clamp(d->z / w->depth, 0.0, 1.0);
  state_out[drone_offset + 3] = clamp(d->x_dot / MAX_VELOCITY, -1.0, 1.0);
  state_out[drone_offset + 4] = clamp(d->y_dot / MAX_VELOCITY, -1.0, 1.0);
  state_out[drone_offset + 5] = clamp(d->z_dot / MAX_VELOCITY, -1.0, 1.0);
  state_out[drone_offset + 6] = clamp(d->phi / ANGLE_LIMIT, -1.0, 1.0);
  state_out[drone_offset + 7] = clamp(d->theta / ANGLE_LIMIT, -1.0, 1.0);
  state_out[drone_offset + 8] = clamp(d->psi / MATH_PI, -1.0, 1.0);
  state_out[drone_offset + 9] = clamp(d->p / MAX_ROT, -1.0, 1.0);
  state_out[drone_offset + 10] = clamp(d->q / MAX_ROT, -1.0, 1.0);
  state_out[drone_offset + 11] = clamp(d->r / MAX_ROT, -1.0, 1.0);
}

/* Fais un pas pour calculer le prochain état physique */
void envStep(Env *env, double *next_state, double *reward, int *is_terminal,
             int action_idx) {
  World *w = env->physical_world;
  Drone *d = w->drone;
  double accumulated_reward = 0.0;
  int crashed = 0;

  // Si la batterie est déjà à 0, on stoppe immédiatement (crash ou fin de
  // mission réussie) et on calcule la récompense associée.
  if (d->battery_remaining == 0) {
    getStateVector(env, next_state);
    *reward = getBatteryReward(env);
    *is_terminal = 1;
    return;
  }

  // On consomme une unité de batterie pour cette action.
  // Le drone peut encore exécuter cette action, mais il ne pourra plus en faire
  // d'autre ensuite.
  double battery_reward = getBatteryReward(env);
  d->battery_remaining--;

  handleCommand(w->drone, action_idx);

  // On laisse le drone exécuter l'action choisie pendant FRAME_SKIP itérations
  // physiques
  for (int i = 0; i < FRAME_SKIP; i++) {
    physicsStep(w, DT);
    crashed = isDroneCrashed(w);
    accumulated_reward += getReward(env);
    if (crashed)
      break;
  }

  getStateVector(env, next_state);

  double final_reward = accumulated_reward / FRAME_SKIP;
  final_reward += battery_reward;

  if (crashed) {
    final_reward -= 100.0;
  }
  *reward = final_reward;
  *is_terminal = crashed;
}

/* Initialise l'environnement en capturant la configuration dynamique du monde
 */
Env *initEnv(World *w, int max_steps) {
  Env *env = malloc(sizeof(Env));

  env->physical_world = w;
  env->current_reward = 0.0;
  env->is_terminal = 0;
  env->max_steps = max_steps;
  w->drone->battery_remaining = max_steps;

  // Sauvegarde de la position de départ du drone
  env->spawn_drone_x = w->drone->x;
  env->spawn_drone_y = w->drone->y;
  env->spawn_drone_z = w->drone->z;

  // Allocation des tableaux de sauvegarde pour les utilisateurs
  env->spawn_users_x = malloc(w->numUsers * sizeof(double));
  env->spawn_users_y = malloc(w->numUsers * sizeof(double));
  for (int i = 0; i < w->numUsers; i++) {
    env->spawn_users_x[i] = w->users[i].x;
    env->spawn_users_y[i] = w->users[i].y;
  }

  // Allocation des tableaux de sauvegarde pour les obstacles
  env->spawn_obs_x = malloc(w->numObstacles * sizeof(double));
  env->spawn_obs_y = malloc(w->numObstacles * sizeof(double));
  for (int i = 0; i < w->numObstacles; i++) {
    env->spawn_obs_x[i] = w->obstacles[i].x;
    env->spawn_obs_y[i] = w->obstacles[i].y;
  }

  getStateVector(env, env->current_state);

  return env;
}

/* Réinitialise l'environnement en se basant UNIQUEMENT sur les paramètres
 * capturés */
void resetEnv(Env *env, int current_epoch) {
  World *w = env->physical_world;

  // Réinitialisation du drone à sa vraie position d'origine.
  // La batterie est réinitialisée à max_steps pour chaque nouvelle simulation.
  *(w->drone) =
      createDrone(env->spawn_drone_x, env->spawn_drone_y, env->spawn_drone_z);
  w->drone->battery_remaining = env->max_steps;
  env->current_reward = 0.0;
  env->is_terminal = 0;

  if (current_epoch < 200) {
    // Mode Fixe
    majWorld(w, NO_RAND);

  } else if (current_epoch < 500 && current_epoch % 30 == 0) {
    // Mode Bruit
    majWorld(w, LOW_RAND);

  } else if (current_epoch % 30 == 0) {
    // Mode Aléatoire Total
    majWorld(w, TOTAL_RAND);
  }

  getStateVector(env, env->current_state);
}
