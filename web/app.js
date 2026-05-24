const canvasProfile = document.getElementById('canvasProfile');
const ctxProfile = canvasProfile.getContext('2d');
const canvasTop = document.getElementById('canvasTop');
const ctxTop = canvasTop.getContext('2d');

// Dimensions physiques (Synchronisées avec ton C)
const physSpanX = 200; // Axe X physique = Avancement (0 à 200)
const physSpanY = 200; // Axe Y physique = Gauche/Droite (0 à 200) -> MAJ
const physSpanZ = 100; // Axe Z physique = Altitude (0 à 100)
const gridSpacing = 20;
const trail = [];

/* --- MAGIE DU MAPPING (Correction de l'inversion) --- */

// En physique : Y augmente vers la gauche. 
// En visuel : On veut que 0 soit à gauche et 200 à droite.
// Solution : Visuel_Y = 200 - Physique_Y
function getVisualX(physY, canvas) { 
    let visualY = physSpanY - physY; 
    return (visualY / physSpanY) * canvas.width; 
}

// Altitude (Z) : 0 en bas, max en haut
function getVisualZ(physZ, canvas) { 
    return canvas.height - ((physZ / physSpanZ) * canvas.height); 
}

// Avancement (X) : 0 en bas, max en haut
function getVisualY(physX, canvas) { 
    return canvas.height - ((physX / physSpanX) * canvas.height); 
}

function getLinkColor(distance) {
    if (distance < 30) return '#a6e3a1'; 
    if (distance < 70) return '#f9e2af'; 
    return '#f38ba8';                    
}

function drawGridProfile() {
    ctxProfile.strokeStyle = "#45475a"; ctxProfile.lineWidth = 0.5;
    ctxProfile.font = "11px 'Segoe UI'"; ctxProfile.fillStyle = "#bac2de";
    
    // Lignes verticales (Gauche/Droite visuel)
    // S'adapte automatiquement grâce à physSpanY
    for (let v = 0; v <= physSpanY; v += gridSpacing) {
        let px = (v / physSpanY) * canvasProfile.width;
        ctxProfile.beginPath(); ctxProfile.moveTo(px, 0); ctxProfile.lineTo(px, canvasProfile.height); ctxProfile.stroke();
        ctxProfile.fillText(v + "m", px + 5, canvasProfile.height - 10);
    }
    // Lignes horizontales (Altitude Z)
    for (let z = 0; z <= physSpanZ; z += gridSpacing) {
        let py = getVisualZ(z, canvasProfile);
        ctxProfile.beginPath(); ctxProfile.moveTo(0, py); ctxProfile.lineTo(canvasProfile.width, py); ctxProfile.stroke();
        ctxProfile.fillText(z + "m", 10, py - 5);
    }
    ctxProfile.font = "bold 13px 'Segoe UI'"; ctxProfile.fillStyle = "#89b4fa";
    ctxProfile.fillText("Z (Altitude)", 15, 25);
    
    // Aligné proprement au centre bas pour éviter les collisions avec les chiffres de la grille
    ctxProfile.textAlign = "center";
    ctxProfile.fillText("Y (Gauche/Droite)", canvasProfile.width / 2, canvasProfile.height - 25);
    ctxProfile.textAlign = "left"; // Reset
}

function drawGridTop() {
    ctxTop.strokeStyle = "#45475a"; ctxTop.lineWidth = 0.5;
    ctxTop.font = "11px 'Segoe UI'"; ctxTop.fillStyle = "#bac2de";
    
    // Lignes verticales (Gauche/Droite visuel)
    for (let v = 0; v <= physSpanY; v += gridSpacing) {
        let px = (v / physSpanY) * canvasTop.width;
        ctxTop.beginPath(); ctxTop.moveTo(px, 0); ctxTop.lineTo(px, canvasTop.height); ctxTop.stroke();
        ctxTop.fillText(v + "m", px + 5, canvasTop.height - 10);
    }
    // Lignes horizontales (Avant/Arrière X)
    for (let x = 0; x <= physSpanX; x += gridSpacing) {
        let py = getVisualY(x, canvasTop);
        ctxTop.beginPath(); ctxTop.moveTo(0, py); ctxTop.lineTo(canvasTop.width, py); ctxTop.stroke();
        ctxTop.fillText(x + "m", 10, py - 5);
    }
    ctxTop.font = "bold 13px 'Segoe UI'"; ctxTop.fillStyle = "#89b4fa";
    ctxTop.fillText("X (Avant/Arrière)", 15, 25);
    
    // Aligné proprement au centre bas également
    ctxTop.textAlign = "center";
    ctxTop.fillText("Y (Gauche/Droite)", canvasTop.width / 2, canvasTop.height - 25);
    ctxTop.textAlign = "left"; // Reset
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

    // ==========================================
    // 1. OBSTACLES & SCAN LiDAR
    // ==========================================
    if (data.obstacles) {
        data.obstacles.forEach(obs => {
            obs.distSq = Math.pow(data.drone.x - obs.x, 2) + Math.pow(data.drone.y - obs.y, 2) + Math.pow(data.drone.z - obs.z, 2);
            obs.isClosest = false;
        });

        let closestObstacles = [...data.obstacles].sort((a, b) => a.distSq - b.distSq).slice(0, 3);
        closestObstacles.forEach(obs => obs.isClosest = true);

        const time = Date.now();
        const blinkAlpha = 0.7 + 0.3 * Math.sin(time / 150); 

        data.obstacles.forEach(obs => {
            let cxTop = getVisualX(obs.y, canvasTop);
            let cyTop = getVisualY(obs.x, canvasTop);
            let visualRadius = (obs.radius / physSpanY) * canvasTop.width; // S'adapte au nouveau physSpanY
            
            let cxProf = getVisualX(obs.y, canvasProfile);
            let bottomZ = getVisualZ(obs.z, canvasProfile);
            let topZ = getVisualZ(obs.z + obs.height, canvasProfile);
            let widthProf = (obs.radius * 2 / physSpanY) * canvasProfile.width; // S'adapte au nouveau physSpanY
            let heightProf = bottomZ - topZ;

            ctxTop.fillStyle = 'rgba(166, 227, 161, 0.15)'; 
            ctxTop.strokeStyle = 'rgba(166, 227, 161, 0.4)';
            ctxTop.lineWidth = 1;

            ctxProfile.fillStyle = 'rgba(166, 227, 161, 0.15)';
            ctxProfile.strokeStyle = 'rgba(166, 227, 161, 0.4)';
            ctxProfile.lineWidth = 1;

            if (obs.isClosest) {
                ctxTop.strokeStyle = `rgba(249, 226, 175, ${blinkAlpha})`;
                ctxTop.lineWidth = 2;
                ctxTop.shadowColor = '#f9e2af';
                ctxTop.shadowBlur = 15 * blinkAlpha;

                ctxProfile.strokeStyle = `rgba(249, 226, 175, ${blinkAlpha})`;
                ctxProfile.lineWidth = 2;
                ctxProfile.shadowColor = '#f9e2af';
                ctxProfile.shadowBlur = 15 * blinkAlpha;
            }

            ctxTop.beginPath();
            ctxTop.arc(cxTop, cyTop, visualRadius, 0, Math.PI*2);
            ctxTop.fill();
            ctxTop.stroke();
            ctxTop.shadowBlur = 0; 

            ctxProfile.beginPath();
            ctxProfile.rect(cxProf - widthProf/2, topZ, widthProf, heightProf);
            ctxProfile.fill();
            ctxProfile.stroke();
            ctxProfile.shadowBlur = 0; 
        });
    }

    // ==========================================
    // 2. LIENS WIFI
    // ==========================================
    data.users.forEach(user => {
        const dist3D = Math.sqrt(
            Math.pow(data.drone.x - user.x, 2) + 
            Math.pow(data.drone.y - user.y, 2) + 
            Math.pow(data.drone.z - user.z, 2)
        );
        const color = getLinkColor(dist3D);

        ctxProfile.beginPath();
        ctxProfile.moveTo(getVisualX(data.drone.y, canvasProfile), getVisualZ(data.drone.z, canvasProfile));
        ctxProfile.lineTo(getVisualX(user.y, canvasProfile), getVisualZ(user.z, canvasProfile));
        ctxProfile.strokeStyle = color; ctxProfile.lineWidth = 1.5; ctxProfile.setLineDash([5, 5]); ctxProfile.stroke();
        
        ctxTop.beginPath();
        ctxTop.moveTo(getVisualX(data.drone.y, canvasTop), getVisualY(data.drone.x, canvasTop));
        ctxTop.lineTo(getVisualX(user.y, canvasTop), getVisualY(user.x, canvasTop));
        ctxTop.strokeStyle = color; ctxTop.lineWidth = 1.5; ctxTop.setLineDash([5, 5]); ctxTop.stroke();
        
        ctxProfile.setLineDash([]); ctxTop.setLineDash([]);
    });

    // ==========================================
    // 3. TRAÎNÉE DU DRONE
    // ==========================================
    ctxProfile.beginPath(); ctxTop.beginPath();
    for (let i = 0; i < trail.length; i++) {
        const p = trail[i];
        const alpha = i / trail.length;
        
        ctxProfile.lineTo(getVisualX(p.y, canvasProfile), getVisualZ(p.z, canvasProfile));
        ctxProfile.strokeStyle = `rgba(137, 220, 235, ${alpha})`;
        
        ctxTop.lineTo(getVisualX(p.y, canvasTop), getVisualY(p.x, canvasTop));
        ctxTop.strokeStyle = `rgba(137, 220, 235, ${alpha})`;
    }
    ctxProfile.stroke(); ctxTop.stroke();

    // ==========================================
    // 4. UTILISATEURS (Piétons)
    // ==========================================
    ctxProfile.fillStyle = '#fab387'; ctxTop.fillStyle = '#fab387';
    data.users.forEach(u => {
        ctxProfile.beginPath(); ctxProfile.arc(getVisualX(u.y, canvasProfile), getVisualZ(u.z, canvasProfile), 6, 0, Math.PI*2); ctxProfile.fill();
        ctxTop.beginPath(); ctxTop.arc(getVisualX(u.y, canvasTop), getVisualY(u.x, canvasTop), 6, 0, Math.PI*2); ctxTop.fill();
    });

    // ==========================================
    // 5. DRONE 
    // ==========================================
    ctxProfile.save();
    ctxProfile.translate(getVisualX(data.drone.y, canvasProfile), getVisualZ(data.drone.z, canvasProfile));
    ctxProfile.rotate(data.drone.phi);
    ctxProfile.fillStyle = '#89dceb'; ctxProfile.fillRect(-20, -2, 40, 4); 
    ctxProfile.fillStyle = '#f38ba8'; ctxProfile.beginPath(); ctxProfile.arc(0, 0, 4, 0, Math.PI*2); ctxProfile.fill();
    ctxProfile.restore();

    ctxTop.save();
    ctxTop.translate(getVisualX(data.drone.y, canvasTop), getVisualY(data.drone.x, canvasTop));
    ctxTop.rotate(data.drone.psi);
    ctxTop.fillStyle = '#a6e3a1'; ctxTop.fillRect(-15, -2, 30, 4); 
    ctxTop.fillStyle = '#89dceb'; ctxTop.fillRect(-2, -15, 4, 30); 
    ctxTop.fillStyle = '#f38ba8'; ctxTop.beginPath(); ctxTop.arc(0, -15, 4, 0, Math.PI*2); ctxTop.fill();
    ctxTop.fillStyle = '#cdd6f4'; ctxTop.beginPath(); ctxTop.arc(0, 0, 5, 0, Math.PI*2); ctxTop.fill();
    ctxTop.restore();
}

document.getElementById('btnLaunch').addEventListener('click', () => {
    // On envoie une requête POST au serveur pour lui dire de lancer l'exécutable
    fetch('/api/launch-exe', { method: 'POST' })
        .then(response => {
            if (response.ok) {
                alert("Script exécuté avec succès !");
            } else {
                alert("Erreur lors du lancement du script.");
            }
        })
        .catch(err => console.error("Impossible de joindre le serveur:", err));
});

setInterval(() => {
    fetch('state.json', { cache: 'no-store' })
        .then(r => r.json())
        .then(data => drawViews(data))
        .catch(() => {});
}, 30);