import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.fft import fft, fftfreq
from piston_anim import Piston 


R = 1.0
L = 3.0
dt = 0.05
frames = 200

fig = plt.figure(figsize=(12, 8))
gs = fig.add_gridspec(3, 2, height_ratios=[1, 1, 1])

# Axe 3D
ax3d = fig.add_subplot(gs[:, 0], projection='3d')
ax3d.set_ylim(-3, 3)
ax3d.set_zlim(-1, 4)
ax3d.set_xlabel("X")
ax3d.set_ylabel("Y")
ax3d.set_zlabel("Z")
ax3d.set_title("Moteur 3D")

# Axes 2D
ax_pos = fig.add_subplot(gs[0, 1])
ax_vit = fig.add_subplot(gs[1, 1])
ax_acc = fig.add_subplot(gs[2, 1])

# Création de 4 pistons avec phases décalées
phases = [0, np.pi, np.pi, 0, np.pi/2, 0]
pistons = []
x_positions = [i for i in range(len(phases))]
ax3d.set_xlim(-1, len(phases))

for i in range(len(phases)):
    piston = Piston(R, L, dt, frames, phases[i], ax3d, ax_pos, ax_vit, ax_acc, x_pos=x_positions[i])
    piston.calc_dynamics()
    pistons.append(piston)


# Calcul des limites y pour vitesse et acceleration avec harmoniques
pos_all = []
vit_all = []
acc_all = []

for p in pistons:
    pos_all.append(p.position)
    vit_all.append(p.vitesse)
    vit_all.extend(p.harmo_vit)
    acc_all.append(p.acceleration)
    acc_all.extend(p.harmo_acc)

pos_all = np.concatenate(pos_all)
vit_all = np.concatenate(vit_all)
acc_all = np.concatenate(acc_all)

ax_pos.set_ylim(np.min(pos_all)-0.5, np.max(pos_all)+0.5)
ax_vit.set_ylim(np.min(vit_all)-0.5, np.max(vit_all)+0.5)
ax_acc.set_ylim(np.min(acc_all)-0.5, np.max(acc_all)+0.5)

# Fonction d'update pour animation
def update_all(frame):
    artists = []
    for p in pistons:
        artists += p.update(frame)
    return artists

ani = FuncAnimation(fig, update_all, frames=frames, interval=50, blit=False)

########### Pour sauvegarder le GIF, Dé-commentez !###########
# Pour enregistrer le GIF sous forme de GIF, il vous suffit d'enlever le "#" devant la ligne ci-dessous
# ani.save(f'bielle_manivelle_{len(phases)}_cylindres.gif', writer=PillowWriter(fps=20))

plt.tight_layout()
plt.show()
