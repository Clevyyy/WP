# Système bielle-manivelle
Ce code permet de générer N bielles-manivelles avec un déphasage au choix sur la ligne suivante.
`phases = [0, np.pi, np.pi, 0, np.pi/2, 0]`
Cette liste correspond à un système de 6 pistons orienté tel que: 0°, 180°, 180°, 0°, 90°, 0°

Il génère ensuite un GIF avec les pistons, ainsi que la position, la vitesse et l'accélération, et leurs deux harmoniques (via FFT).
![](bielle_manivelle_6_cylindres.gif)

*Suite à venir*
