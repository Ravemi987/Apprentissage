const canvasProfile = document.getElementById('canvasProfile');
const ctxProfile = canvasProfile.getContext('2d');
const canvasTop = document.getElementById('canvasTop');
const ctxTop = canvasTop.getContext('2d');

// Dimensions physiques (Synchronisées avec simulation.h)
const physSpanX = 200; // Axe X physique = Avancement (Contrôlé par le Pitch)
const physSpanY = 100; // Axe Y physique = Gauche/Droite (Contrôlé par le Roll)
const physSpanZ = 100; // Axe Z physique = Altitude (Contrôlé par le Thrust)
const gridSpacing = 20;
const trail = [];

/* --- FONCTIONS DE MAPPING : PHYSIQUE -> PIXELS --- */

// 1. Vue de Profil (Vue depuis l'arrière du drone)
function getProfileX(physY) { 
    // Roll Droite -> Y diminue. On veut l'afficher vers la droite de l'écran.
    return ((physSpanY - physY) / physSpanY) * canvasProfile.width; 
}
function getProfileY(physZ) { 
    // Altitude Z augmente -> Monte vers le haut de l'écran.
    return canvasProfile.height - ((physZ / physSpanZ) * canvasProfile.height); 
}

// 2. Vue de Dessus
function getTopX(physY) { 
    // Mouvement latéral Gauche/Droite (Identique à la vue de profil)
    return ((physSpanY - physY) / physSpanY) * canvasTop.width; 
}
function getTopY(physX) { 
    // Pitch Avant -> X augmente. On veut l'afficher vers le haut de l'écran.
    return canvasTop.height - ((physX / physSpanX) * canvasTop.height); 
}


function getLinkColor(distance) {
    if (distance < 30) return '#a6e3a1'; 
    if (distance < 70) return '#f9e2af'; 
    return '#f38ba8';                    
}

function drawGridProfile() {
    ctxProfile.strokeStyle = "#45475a"; ctxProfile.lineWidth = 0.5;
    ctxProfile.font = "11px 'Segoe UI'"; ctxProfile.fillStyle = "#bac2de";
    
    // Lignes verticales (Axe Gauche/Droite visuel : on force 0 à gauche et max à droite)
    for (let i = 0; i <= physSpanY; i += gridSpacing) {
        let px = (i / physSpanY) * canvasProfile.width;
        ctxProfile.beginPath(); ctxProfile.moveTo(px, 0); ctxProfile.lineTo(px, canvasProfile.height); ctxProfile.stroke();
        ctxProfile.fillText(i + "m", px + 5, canvasProfile.height - 10);
    }

    // Lignes horizontales (Altitude Z : 0 en bas, max en haut)
    for (let z = 0; z <= physSpanZ; z += gridSpacing) {
        let py = getProfileY(z);
        ctxProfile.beginPath(); ctxProfile.moveTo(0, py); ctxProfile.lineTo(canvasProfile.width, py); ctxProfile.stroke();
        ctxProfile.fillText(z + "m", 10, py - 5);
    }

    ctxProfile.font = "bold 13px 'Segoe UI'"; ctxProfile.fillStyle = "#89b4fa";
    ctxProfile.fillText("Z (Altitude)", 15, 25);
    ctxProfile.fillText("Y (Gauche/Droite)", canvasProfile.width - 120, canvasProfile.height - 15);
}

function drawGridTop() {
    ctxTop.strokeStyle = "#45475a"; ctxTop.lineWidth = 0.5;
    ctxTop.font = "11px 'Segoe UI'"; ctxTop.fillStyle = "#bac2de";
    
    // Lignes verticales (Axe Gauche/Droite visuel : on force 0 à gauche et max à droite)
    for (let i = 0; i <= physSpanY; i += gridSpacing) {
        let px = (i / physSpanY) * canvasTop.width;
        ctxTop.beginPath(); ctxTop.moveTo(px, 0); ctxTop.lineTo(px, canvasTop.height); ctxTop.stroke();
        ctxTop.fillText(i + "m", px + 5, canvasTop.height - 10);
    }

    // Lignes horizontales (Axe X physique, Avant/Arrière : 0 en bas, max en haut)
    for (let x = 0; x <= physSpanX; x += gridSpacing) {
        let py = getTopY(x);
        ctxTop.beginPath(); ctxTop.moveTo(0, py); ctxTop.lineTo(canvasTop.width, py); ctxTop.stroke();
        ctxTop.fillText(x + "m", 10, py - 5);
    }

    ctxTop.font = "bold 13px 'Segoe UI'"; ctxTop.fillStyle = "#89b4fa";
    ctxTop.fillText("X (Avant/Arrière)", 15, 25);
    ctxTop.fillText("Y (Gauche/Droite)", canvasTop.width - 120, canvasTop.height - 15);
}

function drawViews(data) {
    if (!data.drone || !data.users) return;

    // Mise à jour de la traînée
    trail.push({x: data.drone.x, y: data.drone.y, z: data.drone.z});
    if (trail.length > 80) trail.shift();

    ctxProfile.clearRect(0, 0, canvasProfile.width, canvasProfile.height);
    ctxTop.clearRect(0, 0, canvasTop.width, canvasTop.height);

    drawGridProfile();
    drawGridTop();

    // Dessin des Liens WiFi
    data.users.forEach(user => {
        const dist3D = Math.sqrt(
            Math.pow(data.drone.x - user.x, 2) + 
            Math.pow(data.drone.y - user.y, 2) + 
            Math.pow(data.drone.z - user.z, 2)
        );
        const color = getLinkColor(dist3D);

        // Lien Profil
        ctxProfile.beginPath();
        ctxProfile.moveTo(getProfileX(data.drone.y), getProfileY(data.drone.z));
        ctxProfile.lineTo(getProfileX(user.y), getProfileY(user.z));
        ctxProfile.strokeStyle = color; ctxProfile.lineWidth = 1.5; ctxProfile.setLineDash([5, 5]); ctxProfile.stroke();
        
        // Lien Dessus
        ctxTop.beginPath();
        ctxTop.moveTo(getTopX(data.drone.y), getTopY(data.drone.x));
        ctxTop.lineTo(getTopX(user.y), getTopY(user.x));
        ctxTop.strokeStyle = color; ctxTop.lineWidth = 1.5; ctxTop.setLineDash([5, 5]); ctxTop.stroke();
        
        ctxProfile.setLineDash([]); ctxTop.setLineDash([]);
    });

    // Dessin de la Traînée
    ctxProfile.beginPath(); ctxTop.beginPath();
    for (let i = 0; i < trail.length; i++) {
        const p = trail[i];
        const alpha = i / trail.length;
        
        ctxProfile.lineTo(getProfileX(p.y), getProfileY(p.z));
        ctxProfile.strokeStyle = `rgba(137, 220, 235, ${alpha})`;
        
        ctxTop.lineTo(getTopX(p.y), getTopY(p.x));
        ctxTop.strokeStyle = `rgba(137, 220, 235, ${alpha})`;
    }
    ctxProfile.stroke(); ctxTop.stroke();

    // Dessin des Utilisateurs
    ctxProfile.fillStyle = '#fab387'; ctxTop.fillStyle = '#fab387';
    data.users.forEach(u => {
        ctxProfile.beginPath(); ctxProfile.arc(getProfileX(u.y), getProfileY(u.z), 7, 0, Math.PI*2); ctxProfile.fill();
        ctxTop.beginPath(); ctxTop.arc(getTopX(u.y), getTopY(u.x), 7, 0, Math.PI*2); ctxTop.fill();
    });

    // --- DRONE : VUE DE PROFIL (Vue de l'arrière) ---
    ctxProfile.save();
    ctxProfile.translate(getProfileX(data.drone.y), getProfileY(data.drone.z));
    // Rotation liée au ROLL (phi)
    ctxProfile.rotate(data.drone.phi); 
    ctxProfile.fillStyle = '#89dceb'; ctxProfile.fillRect(-20, -2, 40, 4); 
    ctxProfile.fillStyle = '#f38ba8'; ctxProfile.beginPath(); ctxProfile.arc(0, 0, 4, 0, Math.PI*2); ctxProfile.fill();
    ctxProfile.restore();

    // --- DRONE : VUE DE DESSUS ---
    ctxTop.save();
    ctxTop.translate(getTopX(data.drone.y), getTopY(data.drone.x));
    // Rotation liée au YAW (psi). Inversé car le Canvas tourne à l'envers des repères mathématiques.
    ctxTop.rotate(-data.drone.psi); 
    
    // Bras Gauche/Droite
    ctxTop.fillStyle = '#a6e3a1'; 
    ctxTop.fillRect(-15, -2, 30, 4); 
    // Bras Avant/Arrière
    ctxTop.fillStyle = '#89dceb'; 
    ctxTop.fillRect(-2, -15, 4, 30); 
    // Tête du drone (Rouge) pour voir vers où il pointe
    ctxTop.fillStyle = '#f38ba8'; 
    ctxTop.beginPath(); ctxTop.arc(0, -15, 4, 0, Math.PI*2); ctxTop.fill();
    
    ctxTop.fillStyle = '#cdd6f4'; // Centre
    ctxTop.beginPath(); ctxTop.arc(0, 0, 5, 0, Math.PI*2); ctxTop.fill();
    ctxTop.restore();
}

setInterval(() => {
    fetch('state.json', { cache: 'no-store' })
        .then(r => r.json())
        .then(data => drawViews(data))
        .catch(() => {});
}, 30);
