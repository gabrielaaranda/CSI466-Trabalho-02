import pandas as pd
import math


def readSubject(nomeDoCurso):
    df = pd.read_csv(nomeDoCurso)
    grafo = {}

    # Adiciona o nó s
    grafo['s'] = {'nome': 'Nó Inicial', 'adjacencias': {}}
    for i, row in df.iterrows():
        codigo = row['Código']
        nome = row['Nome']
        duracao = row['Duração']
        dependencias = row['Dependências']
        

        if codigo != 's' and pd.isna(df.loc[df['Código'] == codigo, 'Dependências'].iloc[0]):
            grafo['s']['adjacencias'][codigo] = 0

        if codigo != 's' and pd.isna(df.loc[df['Código'] == codigo, 'Dependências'].iloc[0]):
            grafo['s']['adjacencias'][codigo] = 0

        # Adiciona o nó
        if codigo not in grafo:
            grafo[codigo] = {'nome': nome, 'adjacencias': {}}
        
        # Adiciona as arestas e seus pesos
        if not pd.isna(dependencias):
            dependencias = dependencias.split(',')

            for dep in dependencias:
                dep = dep.strip()
                if dep not in grafo:
                    grafo[dep] = {'nome': '', 'adjacencias': {}}
                grafo[codigo]['adjacencias'][dep] = duracao
            
    # Adiciona o nó t e as arestas do restante do grafo para o nó t
    grafo['t'] = {'nome': 'Nó Destino', 'adjacencias': {}}
    for node in grafo:
        if node != 's' and node != 't':
            grafo[node]['adjacencias']['t'] = 1

    print(grafo)

#    print(max_path_dijkstra(grafo, grafo['s']))

def dijkstra(grafo, s):
    dist = [float("inf")] * len(grafo)
    pred = [-1] * len(grafo)
    dist[s] = 0

    Q = []
    for node in range(grafo):
        Q.append(node)

    while Q != []:
        u = min(Q, key=lambda node: dist[node])
        
        if u != -1:  
            Q.remove(u)
            for v, w in grafo[u]['adjacencias'].items():
                if dist[v] < dist[u] + w:
                    dist[v] = dist[u] + w
                    pred[v] = u

    return dist, pred


def max_path_dijkstra(graph, source):
    dist = {v: -math.inf for v in graph}
    pred = {v: None for v in graph}
    dist[source] = 0
    Q = set(graph)

    while Q:
        u = max(Q, key=dist.get)
        Q.remove(u)
        for v in graph[u]:
            weight = graph[u][v]
            if dist[v] < dist[u] - weight:
                dist[v] = dist[u] - weight
                pred[v] = u

    max_dist = dist[max(dist, key=dist.get)]
    path = []
    curr = max(dist, key=dist.get)
    while curr is not None:
        path.append(curr)
        curr = pred[curr]
    path.reverse()

    return path, max_dist