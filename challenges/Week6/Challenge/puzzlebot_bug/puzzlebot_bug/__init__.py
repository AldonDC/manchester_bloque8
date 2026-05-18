"""
puzzlebot_bug — Mini Challenge · Reactive Navigation (Week 6)

Paquete ROS 2 que implementa los algoritmos clásicos Bug 0 y Bug 2 para
navegación reactiva del Puzzlebot en Gazebo Harmonic.

Subpaquetes:
    perception/    — procesamiento del LiDAR (/scan → sectores frontal/izq/der)
    navigation/    — máquinas de estado Bug 0 y Bug 2
    localisation/  — dead reckoning + Σ_k (reutilizado del Reto 4)
    control/       — PID a waypoint + publicador del objetivo
"""
