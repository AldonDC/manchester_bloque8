"""
puzzlebot_sim — Mini Challenge 4 (Week 5)

Paquete ROS 2 que extiende la odometría del Puzzlebot con propagación de
incertidumbre Gaussiana (matriz de covarianza Σₖ) según el modelo del PDF.

Subpaquetes:
    simulation/    — Real/Sim Robot y driver open-loop para experimentos
    localisation/  — Dead reckoning + propagación Σₖ (núcleo del reto)
    visualisation/ — TF dinámica y publicador de joints para RViz
    control/       — Controlador PID y generador de trayectorias
"""
