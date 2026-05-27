"""
control/ — Lazo de control de movimiento.

Nodos:
    controller       — PID hacia un waypoint (reutilizado del Reto 4,
                       el "multi-point controller" del Reto 2).
    goal_publisher   — Publica el objetivo (x_T, y_T) en /set_point una sola
                       vez al arrancar. Es el equivalente al trajectory
                       generator de retos previos, pero con un único punto.
"""
