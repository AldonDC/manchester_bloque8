import graphviz

def generate_architecture_diagram():
    """
    Genera un diagrama de bloques profesional siguiendo la estética de Manchester Robotics.
    """
    dot = graphviz.Digraph('Puzzlebot_Architecture', comment='Arquitectura Reto 3', format='png')
    dot.attr(rankdir='LR', size='12,8', dpi='300')
    dot.attr('node', fontname='Arial')

    # --- Estilos de Nodos (Círculos como en su PDF) ---
    dot.attr('node', shape='ellipse', style='filled', fontcolor='black', penwidth='2')
    
    # Nodos Naranja (Lógica/Procesamiento)
    dot.node('TG', 'Trajectory / Set Point\nGenerator', fillcolor='#FFDAB9', color='#E67E22')
    dot.node('CTRL', 'Controller\n(PID)', fillcolor='#FFDAB9', color='#E67E22')
    dot.node('JSP', 'Joint State\nPublisher', fillcolor='#FFDAB9', color='#E67E22')
    dot.node('RSP', 'Robot State\nPublisher', fillcolor='#FFDAB9', color='#E67E22')
    dot.node('CT', 'Coordinate\nTransform', fillcolor='#FFDAB9', color='#E67E22')
    
    # Nodo Azul (Robot/Simulación)
    dot.node('ROBOT', 'Real / Sim\nRobot', fillcolor='#E1F5FE', color='#03A9F4')
    
    # Nodo Rojo (Estimación/Sensor)
    dot.node('LOC', 'Localisation\n(Dead Reckoning)', fillcolor='#FFEBEE', color='#F44336')
    
    # Nodo RViz (Tipo pantalla)
    dot.node('RVIZ', 'RViz', shape='box', style='filled', fillcolor='#F5F5F5', color='#212121')

    # --- Tópicos (Las conexiones) ---
    dot.attr('edge', fontname='Arial', fontsize='10', color='#757575')
    
    # Lazo de Control
    dot.edge('TG', 'CTRL', label='set_point')
    dot.edge('CTRL', 'ROBOT', label='cmd_vel')
    dot.edge('ROBOT', 'LOC', label='wl, wr')
    dot.edge('LOC', 'CTRL', label='odom', constraint='false', weight='0')
    dot.edge('CTRL', 'TG', label='next_point', constraint='false')
    
    # Rama de Visualización
    dot.edge('LOC', 'JSP', label='odom')
    dot.edge('LOC', 'CT', label='odom')
    dot.edge('JSP', 'RSP', label='joint_states')
    dot.edge('RSP', 'RVIZ', label='robot_description')
    dot.edge('CT', 'RVIZ', label='tf')

    # Guardar imagen
    output_file = dot.render('architecture_diagram', view=False)
    print(f"¡Diagrama generado con éxito! Se guardó como: {output_file}")

if __name__ == '__main__':
    generate_architecture_diagram()
