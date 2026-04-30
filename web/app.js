const canvasXY = document.getElementById('canvasXY');
const ctxXY = canvasXY.getContext('2d');
const canvasXZ = document.getElementById('canvasXZ');
const ctxXZ = canvasXZ.getContext('2d');

// On synchronise les dimensions avec celles du code C (w.width, w.height, w.depth)
const worldWidth = 200; 
const worldHeight = 100;
const worldDepth = 100;
const trail = [];

// Conversion proportionnelle (X mapé sur 200, Y/Z mapés sur 100)
function getPx(val, canvas) { return (val / worldWidth) * canvas.width; }
function getPy(val, maxVal, canvas) { return canvas.height - ((val / maxVal) * canvas.height); }

function getLinkColor(distance) {
    if (distance < 30) return '#a6e3a1'; 
    if (distance < 70) return '#f9e2af'; 
    return '#f38ba8';                    
}

function drawViews(data) {
    if (!data.drone || !data.users) return;

    trail.push({x: data.drone.x, y: data.drone.y, z: data.drone.z});
    if (trail.length > 60) trail.shift();

    ctxXY.clearRect(0, 0, canvasXY.width, canvasXY.height);
    ctxXZ.clearRect(0, 0, canvasXZ.width, canvasXZ.height);

    // Dessin des Liens WiFi
    data.users.forEach(user => {
        const dist3D = Math.sqrt(
            Math.pow(data.drone.x - user.x, 2) + 
            Math.pow(data.drone.y - user.y, 2) + 
            Math.pow(data.drone.z - user.z, 2)
        );
        const color = getLinkColor(dist3D);

        // Lien XY
        ctxXY.beginPath();
        ctxXY.moveTo(getPx(data.drone.x, canvasXY), getPy(data.drone.y, worldHeight, canvasXY));
        ctxXY.lineTo(getPx(user.x, canvasXY), getPy(user.y, worldHeight, canvasXY));
        ctxXY.strokeStyle = color; ctxXY.lineWidth = 2; ctxXY.setLineDash([5, 5]); ctxXY.stroke();
        
        // Lien XZ
        ctxXZ.beginPath();
        ctxXZ.moveTo(getPx(data.drone.x, canvasXZ), getPy(data.drone.z, worldDepth, canvasXZ));
        ctxXZ.lineTo(getPx(user.x, canvasXZ), getPy(user.z, worldDepth, canvasXZ));
        ctxXZ.strokeStyle = color; ctxXZ.lineWidth = 2; ctxXZ.setLineDash([5, 5]); ctxXZ.stroke();
        
        ctxXY.setLineDash([]); ctxXZ.setLineDash([]);
    });

    // Dessin de la Traînée
    ctxXY.beginPath(); ctxXZ.beginPath();
    for (let i = 0; i < trail.length; i++) {
        const p = trail[i];
        const alpha = i / trail.length;
        
        ctxXY.lineTo(getPx(p.x, canvasXY), getPy(p.y, worldHeight, canvasXY));
        ctxXY.strokeStyle = `rgba(137, 220, 235, ${alpha})`;
        
        ctxXZ.lineTo(getPx(p.x, canvasXZ), getPy(p.z, worldDepth, canvasXZ));
        ctxXZ.strokeStyle = `rgba(137, 220, 235, ${alpha})`;
    }
    ctxXY.stroke(); ctxXZ.stroke();

    // Dessin des Utilisateurs
    ctxXY.fillStyle = '#fab387'; ctxXZ.fillStyle = '#fab387';
    data.users.forEach(u => {
        ctxXY.beginPath(); ctxXY.arc(getPx(u.x, canvasXY), getPy(u.y, worldHeight, canvasXY), 8, 0, Math.PI*2); ctxXY.fill();
        ctxXZ.beginPath(); ctxXZ.arc(getPx(u.x, canvasXZ), getPy(u.z, worldDepth, canvasXZ), 8, 0, Math.PI*2); ctxXZ.fill();
    });

    // Dessin du Drone (Vue Profil)
    ctxXY.save();
    ctxXY.translate(getPx(data.drone.x, canvasXY), getPy(data.drone.y, worldHeight, canvasXY));
    ctxXY.rotate(data.drone.roll); 
    ctxXY.fillStyle = '#89dceb'; ctxXY.fillRect(-25, -3, 50, 6); 
    ctxXY.fillStyle = '#f38ba8'; ctxXY.beginPath(); ctxXY.arc(0, 0, 5, 0, Math.PI*2); ctxXY.fill();
    ctxXY.restore();

    // Dessin du Drone (Vue Dessus)
    ctxXZ.save();
    ctxXZ.translate(getPx(data.drone.x, canvasXZ), getPy(data.drone.z, worldDepth, canvasXZ));
    ctxXZ.fillStyle = '#a6e3a1'; ctxXZ.fillRect(-20, -3, 40, 6); ctxXZ.fillRect(-3, -20, 6, 40);
    ctxXZ.fillStyle = '#f38ba8'; ctxXZ.beginPath(); ctxXZ.arc(0, 0, 6, 0, Math.PI*2); ctxXZ.fill();
    ctxXZ.restore();
}

setInterval(() => {
    fetch('state.json', { cache: 'no-store' })
        .then(r => r.json())
        .then(data => drawViews(data))
        .catch(() => {});
}, 30);
