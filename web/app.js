const canvasXZ = document.getElementById('canvasXZ'); // Vue de Profil (Elevation : X / Z)
const ctxXZ = canvasXZ.getContext('2d');
const canvasXY = document.getElementById('canvasXY'); // Vue de Dessus (Navigation : X / Y)
const ctxXY = canvasXY.getContext('2d');

// Dimensions synchronisées avec simulation.h
// Z est l'altitude, Y est la profondeur
const worldWidth = 200;  // Axe X
const worldHeight = 100; // Axe Y (Profondeur)
const worldDepth = 100;  // Axe Z (Altitude)
const gridSpacing = 20;
const trail = [];

// Conversion proportionnelle
// L'axe X sur le canvas va de gauche à droite
function getPx(val, canvas) { return (val / worldWidth) * canvas.width; }

// Sur un canvas, l'axe Y va vers le bas. On l'inverse pour que 0 soit en bas.
function getPy(val, maxVal, canvas) { return canvas.height - ((val / maxVal) * canvas.height); }

function getLinkColor(distance) {
    if (distance < 30) return '#a6e3a1'; 
    if (distance < 70) return '#f9e2af'; 
    return '#f38ba8';                    
}

function drawGrid(ctx, canvas, maxW, maxH, labelHorizontal, labelVertical) {
    ctx.strokeStyle = "#45475a";
    ctx.lineWidth = 0.5;
    ctx.font = "11px 'Segoe UI'"; 
    ctx.fillStyle = "#bac2de";

    // Lignes verticales
    for (let x = 0; x <= maxW; x += gridSpacing) {
        let px = (x / maxW) * canvas.width;
        ctx.beginPath();
        ctx.moveTo(px, 0);
        ctx.lineTo(px, canvas.height);
        ctx.stroke();
        ctx.fillText(x + "m", px + 5, canvas.height - 10);
    }

    // Lignes horizontales
    for (let y = 0; y <= maxH; y += gridSpacing) {
        let py = canvas.height - ((y / maxH) * canvas.height);
        ctx.beginPath();
        ctx.moveTo(0, py);
        ctx.lineTo(canvas.width, py);
        ctx.stroke();
        ctx.fillText(y + "m", 10, py - 5);
    }

    // Étiquettes d'axes
    ctx.font = "bold 13px 'Segoe UI'";
    ctx.fillStyle = "#89b4fa";
    ctx.fillText(labelVertical, 15, 25);
    ctx.fillText(labelHorizontal, canvas.width - 30, canvas.height - 15);
}

function drawViews(data) {
    if (!data.drone || !data.users) return;

    // Mise à jour de la traînée
    trail.push({x: data.drone.x, y: data.drone.y, z: data.drone.z});
    if (trail.length > 80) trail.shift();

    ctxXZ.clearRect(0, 0, canvasXZ.width, canvasXZ.height);
    ctxXY.clearRect(0, 0, canvasXY.width, canvasXY.height);

    // Grilles :
    // Profil : Axe horizontal = X, Axe vertical = Z (Altitude)
    drawGrid(ctxXZ, canvasXZ, worldWidth, worldDepth, "X", "Z (Alt)");
    // Dessus : Axe horizontal = X, Axe vertical = Y (Profondeur)
    drawGrid(ctxXY, canvasXY, worldWidth, worldHeight, "X", "Y (Prof)");

    data.users.forEach(user => {
        const dist3D = Math.sqrt(
            Math.pow(data.drone.x - user.x, 2) + 
            Math.pow(data.drone.y - user.y, 2) + 
            Math.pow(data.drone.z - user.z, 2)
        );
        const color = getLinkColor(dist3D);

        // Lien Vue de Profil (X, Z)
        ctxXZ.beginPath();
        ctxXZ.moveTo(getPx(data.drone.x, canvasXZ), getPy(data.drone.z, worldDepth, canvasXZ));
        ctxXZ.lineTo(getPx(user.x, canvasXZ), getPy(user.z, worldDepth, canvasXZ));
        ctxXZ.strokeStyle = color; ctxXZ.lineWidth = 1.5; ctxXZ.setLineDash([5, 5]); ctxXZ.stroke();
        
        // Lien Vue de Dessus (X, Y)
        ctxXY.beginPath();
        ctxXY.moveTo(getPx(data.drone.x, canvasXY), getPy(data.drone.y, worldHeight, canvasXY));
        ctxXY.lineTo(getPx(user.x, canvasXY), getPy(user.y, worldHeight, canvasXY));
        ctxXY.strokeStyle = color; ctxXY.lineWidth = 1.5; ctxXY.setLineDash([5, 5]); ctxXY.stroke();
        
        ctxXZ.setLineDash([]); ctxXY.setLineDash([]);
    });

    // Dessin de la Traînée
    ctxXZ.beginPath(); ctxXY.beginPath();
    for (let i = 0; i < trail.length; i++) {
        const p = trail[i];
        const alpha = i / trail.length;
        
        ctxXZ.lineTo(getPx(p.x, canvasXZ), getPy(p.z, worldDepth, canvasXZ));
        ctxXZ.strokeStyle = `rgba(137, 220, 235, ${alpha})`;
        
        ctxXY.lineTo(getPx(p.x, canvasXY), getPy(p.y, worldHeight, canvasXY));
        ctxXY.strokeStyle = `rgba(137, 220, 235, ${alpha})`;
    }
    ctxXZ.stroke(); ctxXY.stroke();

    // Dessin des Utilisateurs
    ctxXZ.fillStyle = '#fab387'; ctxXY.fillStyle = '#fab387';
    data.users.forEach(u => {
        ctxXZ.beginPath(); ctxXZ.arc(getPx(u.x, canvasXZ), getPy(u.z, worldDepth, canvasXZ), 7, 0, Math.PI*2); ctxXZ.fill();
        ctxXY.beginPath(); ctxXY.arc(getPx(u.x, canvasXY), getPy(u.y, worldHeight, canvasXY), 7, 0, Math.PI*2); ctxXY.fill();
    });

    // --- Dessin du Drone ---

    // Vue de Profil (Élévation : Axe X et Z)
    // On regarde le drone de face/derrière. L'inclinaison gauche/droite est le ROLL (phi).
    ctxXZ.save();
    ctxXZ.translate(getPx(data.drone.x, canvasXZ), getPy(data.drone.z, worldDepth, canvasXZ));
    // Attention: Sur un canvas, une rotation positive tourne dans le sens horaire.
    // Si phi > 0 (Roll droite), le bras droit s'abaisse, le bras gauche se lève.
    ctxXZ.rotate(data.drone.phi); 
    
    ctxXZ.fillStyle = '#89dceb'; ctxXZ.fillRect(-20, -2, 40, 4); // Axe
    ctxXZ.fillStyle = '#f38ba8'; ctxXZ.beginPath(); ctxXZ.arc(0, 0, 4, 0, Math.PI*2); ctxXZ.fill();
    ctxXZ.restore();

    // Vue de Dessus (Navigation : Axe X et Y)
    ctxXY.save();
    ctxXY.translate(getPx(data.drone.x, canvasXY), getPy(data.drone.y, worldHeight, canvasXY));
    
    // Le Yaw (psi) fait tourner le drone sur lui-même en vue de dessus
    ctxXY.rotate(data.drone.psi); 
    
    // Dessin de la croix (Quadricoptère)
    ctxXY.fillStyle = '#a6e3a1'; 
    ctxXY.fillRect(-15, -2, 30, 4); // Bras gauche/droite
    ctxXY.fillRect(-2, -15, 4, 30); // Bras avant/arrière

    // Corps du drone
    ctxXY.fillStyle = '#f38ba8'; 
    ctxXY.beginPath(); ctxXY.arc(0, 0, 5, 0, Math.PI*2); ctxXY.fill();
    
    // Indicateur de direction (Nez du drone sur l'axe Y)
    ctxXY.fillStyle = '#f9e2af';
    ctxXY.beginPath(); ctxXY.arc(0, -15, 3, 0, Math.PI*2); ctxXY.fill(); 

    ctxXY.restore();
}

setInterval(() => {
    fetch('state.json', { cache: 'no-store' })
        .then(r => r.json())
        .then(data => drawViews(data))
        .catch(() => {});
}, 30);
