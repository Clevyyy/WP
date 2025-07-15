import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.fft import fft, fftfreq

class Piston:
    def __init__(self, R, L, dt, frames, phase, ax3d, ax_pos, ax_vit, ax_acc, x_pos=0):
        self.R = R
        self.L = L
        self.dt = dt
        self.frames = frames
        self.phase = phase
        self.x_pos = x_pos
        
        # Stockage temps
        self.theta = np.linspace(-2*np.pi, 2*np.pi, frames)
        self.theta_deg = np.degrees(self.theta)
        
        self.ax3d = ax3d
        self.ax_pos = ax_pos
        self.ax_vit = ax_vit
        self.ax_acc = ax_acc
        
        # Stockage des données dynamiques
        self.position = np.zeros(frames)
        self.vitesse = np.zeros(frames)
        self.acceleration = np.zeros(frames)
        
        # Initialisation objets graphiques 3D
        self.cyl_piston = None
        self.cyl_bielle = None
        self.cyl_manivelle = None
        
        # Graphiques 2D
        self.line_pos, = ax_pos.plot([], [], label=f'Piston {x_pos}')
        self.line_vit, = ax_vit.plot([], [], label=f'Piston {x_pos}')
        self.line_acc, = ax_acc.plot([], [], label=f'Piston {x_pos}')
        
        # FFT harmoniques (à calculer après le calcul complet)
        self.harmo_vit = None
        self.harmo_acc = None
        self.line_harmo_v = []
        self.line_harmo_a = []
        
        # Setup axes 2D
        for ax in (ax_pos, ax_vit, ax_acc):
            ax.set_xlim(np.min(self.theta_deg), np.max(self.theta_deg))
            ax.grid(True)
        ax_pos.set_title("Position")
        ax_vit.set_title("Vitesse")
        ax_acc.set_title("Accélération")
        ax_pos.set_ylabel("Position")
        ax_vit.set_ylabel("Vitesse")
        ax_acc.set_ylabel("Accélération")
        ax_acc.set_xlabel("Angle (°)")
        
    def pos_piston(self, th):
        x_m = self.R * np.cos(th + self.phase + np.pi/2)
        y_m = self.R * np.sin(th + self.phase + np.pi/2)
        return y_m + np.sqrt(self.L**2 - x_m**2)
    
    def calc_dynamics(self):
        # Calcul position, vitesse, accélération sur toutes les frames
        for i, th in enumerate(self.theta):
            self.position[i] = self.pos_piston(th)
        self.vitesse = np.gradient(self.position, self.dt)
        self.acceleration = np.gradient(self.vitesse, self.dt)
        
        # Analyse FFT
        fft_v = fft(self.vitesse)
        fft_a = fft(self.acceleration)
        freqs = fftfreq(self.frames, d=self.dt)
        
        idx_v = np.argsort(np.abs(fft_v[1:self.frames//2]))[-2:] + 1
        idx_a = np.argsort(np.abs(fft_a[1:self.frames//2]))[-2:] + 1
        
        t = np.linspace(0, self.frames * self.dt, self.frames)
        
        def synth_sin(fft_vals, idx):
            return [(np.abs(fft_vals[i]) / (self.frames/2)) * np.cos(2*np.pi*freqs[i]*t + np.angle(fft_vals[i])) for i in idx]
        
        self.harmo_vit = synth_sin(fft_v, idx_v)
        self.harmo_acc = synth_sin(fft_a, idx_a)
        
        # Préparer lignes harmoniques dans axes 2D
        colors_v = ['orange', 'red']
        colors_a = ['purple', 'magenta']
        
        for i in range(2):
            line_v, = self.ax_vit.plot([], [], '--', color=colors_v[i])
            line_a, = self.ax_acc.plot([], [], '--', color=colors_a[i])
            self.line_harmo_v.append(line_v)
            self.line_harmo_a.append(line_a)
    
    def plot_cylinder(self, ax, x_center, y_center, z_bottom, height, radius, color='b', alpha=0.7):
        # Simplification: cylindre vertical (pas d'inclinaison ici pour aller vite)
        theta_cyl = np.linspace(0, 2*np.pi, 30)
        z_cyl = np.linspace(z_bottom, z_bottom + height, 2)
        theta_grid, z_grid = np.meshgrid(theta_cyl, z_cyl)
        x_grid = x_center + radius * np.cos(theta_grid)
        y_grid = y_center + radius * np.sin(theta_grid)
        return ax.plot_surface(x_grid, y_grid, z_grid, color=color, alpha=alpha, linewidth=0)
    
    def update_3d(self, frame):
        # Supprime anciens plots s'ils existent
        if self.cyl_piston:
            self.cyl_piston.remove()
        if self.cyl_bielle:
            self.cyl_bielle.remove()
        if self.cyl_manivelle:
            self.cyl_manivelle.remove()
    
        z = self.position[frame]
        x = self.x_pos
        r_piston = 0.1
        h_piston = 1
    
        # piston vertical
        self.cyl_piston = self.plot_cylinder(self.ax3d, x, 0, z - 1, h_piston, r_piston, color='blue')
    
        # manivelle tournant dans le plan Y-Z
        angle = self.theta[frame] + self.phase
        r_orbit = self.R
        angle_offset = 0  # tu peux paramétrer ça aussi si besoin
    
        base = [x,
                r_orbit * np.sin(angle + angle_offset),
                r_orbit * np.cos(angle + angle_offset)]
    
        top = [x, 0, z - 1]
    
        # tracé bielle simple en 3D par une ligne (entre base et top)
        self.cyl_bielle = self.ax3d.plot([base[0], top[0]], [base[1], top[1]], [base[2], top[2]], color='green', linewidth=3)[0]
    
        # manivelle (point sur orbite)
        self.cyl_manivelle = self.ax3d.plot([base[0]], [base[1]], [base[2]], marker='o', color='red')[0]
    
    def update_2d(self, frame):
        # Met à jour les courbes 2D (jusqu'à frame)
        self.line_pos.set_data(self.theta_deg[:frame], self.position[:frame])
        self.line_vit.set_data(self.theta_deg[:frame], self.vitesse[:frame])
        self.line_acc.set_data(self.theta_deg[:frame], self.acceleration[:frame])
        
        for i in range(2):
            self.line_harmo_v[i].set_data(self.theta_deg[:frame], self.harmo_vit[i][:frame])
            self.line_harmo_a[i].set_data(self.theta_deg[:frame], self.harmo_acc[i][:frame])
        
    def update(self, frame):
        self.update_3d(frame)
        self.update_2d(frame)
        # Retour des artistes modifiés pour l'animation (utile si blit=True)
        artists = [self.cyl_piston, self.cyl_bielle, self.cyl_manivelle,
                   self.line_pos, self.line_vit, self.line_acc] + self.line_harmo_v + self.line_harmo_a
        return artists
