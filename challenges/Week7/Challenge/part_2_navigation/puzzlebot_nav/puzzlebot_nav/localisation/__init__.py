"""
localisation/ — Dead reckoning + propagación de Σ_k.

Reutilizado del Mini Challenge 4 (Week 5). Estima la pose del Puzzlebot
integrando las velocidades de rueda y propaga la covarianza Σ_k = H·Σ_{k-1}·Hᵀ + Q.
La covarianza creciente es uno de los puntos que pide considerar el reto.
"""
