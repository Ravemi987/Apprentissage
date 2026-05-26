import re
import matplotlib.pyplot as plt

def plot_learning_curve(filename):
    epochs = []
    rewards = []
    
    # Regex pour capturer l'Epoch et le Total Reward, peu importe les espaces
    # Il cherche : "Epoch [chiffres]/[chiffres] | Steps: [chiffres] | Total Reward: [nombres à virgule/négatifs]"
    pattern = re.compile(r"Epoch\s+(\d+)/\d+\s+\|\s+Steps:\s+\d+\s+\|\s+Total Reward:\s+([-]?\d+\.\d+)")
    
    try:
        with open(filename, 'r', encoding='utf-8') as file:
            for line in file:
                match = pattern.search(line)
                if match:
                    epoch = int(match.group(1))
                    reward = float(match.group(2))
                    epochs.append(epoch)
                    rewards.append(reward)
    except FileNotFoundError:
        print(f"Erreur : Le fichier '{filename}' est introuvable.")
        return

    if not epochs:
        print("Aucune donnée trouvée. Vérifie le format du fichier.")
        return

    # Calcul d'une moyenne mobile (lissage) sur 20 epochs
    window_size = 20
    smoothed_rewards = []
    for i in range(len(rewards)):
        start_idx = max(0, i - window_size + 1)
        window = rewards[start_idx:i+1]
        smoothed_rewards.append(sum(window) / len(window))

    # Tracé du graphique
    plt.figure(figsize=(12, 6))
    
    # Courbe brute (transparente pour voir le bruit)
    plt.plot(epochs, rewards, alpha=0.3, color='dodgerblue', label='Récompense brute (bruit)')
    
    # Courbe lissée (en rouge et plus épaisse pour voir la tendance)
    plt.plot(epochs, smoothed_rewards, color='red', linewidth=2.5, label=f'Moyenne mobile (fenêtre={window_size})')
    
    plt.title("Courbe d'apprentissage de l'IA (Deep Q-Learning)", fontsize=14, fontweight='bold')
    plt.xlabel("Epochs", fontsize=12)
    plt.ylabel("Total Reward (Score)", fontsize=12)
    plt.legend(fontsize=12)
    plt.grid(True, linestyle='--', alpha=0.7)
    
    plt.tight_layout()
    plt.show()

# Utilisation : 
# Copie tes logs dans un fichier texte (ex: "logs.txt") et lance la fonction
if __name__ == "__main__":
    plot_learning_curve('files/results.txt')
