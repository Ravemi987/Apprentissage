const canvas = document.getElementById('simCanvas');
const ctx = canvas.getContext('2d');

// Paramètres d'échelle (Supposons que la carte fait 100x100)
const worldWidth = 100;
const worldHeight = 100;
const scaleX = canvas.width / worldWidth;
const scaleY = canvas.height / worldHeight;

// Conversion des coordonnées
function getCanvasX(x) { return x * scaleX; }
function getCanvasY(y) { return canvas.height - (y * scaleY); }

// Fonction pour déterminer la couleur du lien selon la distance physique
function getLinkColor(distance) {
    if (distance < 20) return '#a6e3a1'; // Vert : Excellent
    if (distance < 50) return '#f9e2af'; // Jaune : Moyen
    return '#f38ba8';                    // Rouge : Faible
}

function drawWorld(data) {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    const dx = getCanvasX(data.drone.x);
    const dy = getCanvasY(data.drone.y);

    // 1. Dessiner les liens de connexion (EN PREMIER pour qu'ils soient derrière)
    data.users.forEach(user => {
        const ux = getCanvasX(user.x);
        const uy = getCanvasY(user.y);

        // Calcul de la distance physique (pas en pixels) pour la couleur
        const distance = Math.sqrt(
            Math.pow(data.drone.x - user.x, 2) + 
            Math.pow(data.drone.y - user.y, 2)
        );

        ctx.beginPath();
        ctx.moveTo(dx, dy); // Départ : Drone
        ctx.lineTo(ux, uy); // Arrivée : Utilisateur
        ctx.strokeStyle = getLinkColor(distance);
        ctx.lineWidth = 2;
        ctx.setLineDash([5, 5]); // Ligne pointillée pour simuler des ondes
        ctx.stroke();
        ctx.setLineDash([]); // Reset des pointillés
    });

    // 2. Dessiner les utilisateurs
    ctx.fillStyle = '#fab387'; 
    data.users.forEach(user => {
        ctx.beginPath();
        ctx.arc(getCanvasX(user.x), getCanvasY(user.y), 8, 0, Math.PI * 2);
        ctx.fill();
    });

    // 3. Dessiner le drone (Toujours par-dessus)
    ctx.save();
    ctx.translate(dx, dy);
    ctx.rotate(data.drone.theta); 

    // Corps
    ctx.strokeStyle = '#89dceb';
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(-30, 0);
    ctx.lineTo(30, 0);
    ctx.stroke();

    // Centre
    ctx.fillStyle = '#f38ba8';
    ctx.beginPath();
    ctx.arc(0, 0, 5, 0, Math.PI * 2);
    ctx.fill();

    // Moteurs
    ctx.fillStyle = '#a6e3a1';
    ctx.fillRect(-35, -5, 10, 10);
    ctx.fillRect(25, -5, 10, 10);

    ctx.restore();
}

// Boucle de mise à jour (60 FPS)
setInterval(() => {
    fetch('state.json', { cache: 'no-store' })
        .then(response => response.json())
        .then(data => drawWorld(data))
        .catch(err => {
            // Optionnel : Afficher un message si le C ne tourne pas
            // console.log("Attente de state.json...");
        });
}, 16);
