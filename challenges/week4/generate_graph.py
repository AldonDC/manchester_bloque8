import graphviz

def generate_architecture_graph():
    # Crear un nuevo grafo dirigido
    dot = graphviz.Digraph(comment='Mini Challenge 3 - Multi-Robot Architecture')
    
    # Configuraciones globales de diseño (para que se vea pro y estructurado)
    dot.attr(rankdir='LR', size='12,8')
    dot.attr('node', shape='ellipse', style='filled', color='black', fontname='Arial')
    dot.attr('edge', fontname='Arial', fontsize='10')

    # ── BLOQUE COMÚN (Fuera de los namespaces) ──
    with dot.subgraph(name='cluster_common') as common:
        common.attr(color='white') # Ocultar el borde del cluster común
        common.node('STL', '.STL Files', shape='rect', fillcolor='white', color='orange', penwidth='2')
        common.node('URDF', 'URDF\nFile', shape='rect', fillcolor='white', color='orange', penwidth='2')
        common.node('RViz', 'RViz', shape='rect', fillcolor='black', fontcolor='white')
        
        common.edge('STL', 'URDF')

    # Función para generar el subgraph de un robot
    def create_robot_cluster(graph, ns):
        with graph.subgraph(name=f'cluster_{ns}') as cluster:
            # Estilo de la caja punteada naranja
            cluster.attr(label=f'Namespace: {ns}', style='dashed', color='orange', penwidth='2', fontname='Arial', fontsize='14')
            
            # Definición de Nodos para el robot
            # Colores basados en la imagen (naranja, cian, rojo)
            cluster.node(f'{ns}_controller', f'{ns}/controller', fillcolor='white', color='orange', penwidth='2')
            cluster.node(f'{ns}_robot', f'{ns}/robot', fillcolor='white', color='deepskyblue', penwidth='2')
            cluster.node(f'{ns}_local', f'{ns}/localisation', fillcolor='white', color='red', penwidth='2')
            cluster.node(f'{ns}_jsp', f'{ns}/joint\nstate Pub', fillcolor='white', color='orange', penwidth='2')
            cluster.node(f'{ns}_rsp', f'{ns}/robot\nstate publisher', fillcolor='white', color='orange', penwidth='2')

            # ── Conexiones (Tópicos) internas del robot ──
            # Control -> Planta
            cluster.edge(f'{ns}_controller', f'{ns}_robot', label=f'{ns}/cmd_vel')
            
            # Planta -> Localización
            cluster.edge(f'{ns}_robot', f'{ns}_local', label=f'{ns}/wr,\n{ns}/wl')
            
            # Localización -> Control (Feedback loop)
            # Para que la flecha vuelva por arriba, usamos un constraint o port
            cluster.edge(f'{ns}_local', f'{ns}_controller', label=f'{ns}/odom')
            
            # Localización -> Joint State Publisher
            cluster.edge(f'{ns}_local', f'{ns}_jsp', label=f'{ns}/odom')
            
            # Joint State Publisher -> Robot State Publisher
            cluster.edge(f'{ns}_jsp', f'{ns}_rsp', label=f'{ns}/joint_states')

    # Generar clústeres para Robot 1 y Robot 2
    create_robot_cluster(dot, 'robot1')
    create_robot_cluster(dot, 'robot2')

    # ── Conexiones desde el bloque común hacia los robots ──
    dot.edge('URDF', 'robot1_rsp')
    dot.edge('URDF', 'robot2_rsp')
    
    # ── Conexiones desde los robots hacia RViz ──
    dot.edge('robot1_rsp', 'RViz')
    dot.edge('robot2_rsp', 'RViz')

    # Guardar y renderizar el PDF y PNG
    output_filename = 'multi_robot_architecture'
    dot.render(output_filename, format='png', cleanup=True)
    dot.render(output_filename, format='pdf', cleanup=True)
    
    print(f"¡Diagrama generado exitosamente como {output_filename}.png y .pdf!")

if __name__ == '__main__':
    generate_architecture_graph()
