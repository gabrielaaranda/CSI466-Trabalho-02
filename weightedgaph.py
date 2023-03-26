import pandas as pd
import math

def find_longest_path(file_path):
    # Cria o grafo a partir do arquivo CSV
    df = pd.read_csv(file_path)
    nodes = {}
    for i, row in df.iterrows():
        nodes[row['Código']] = {
            'dependências': row['Dependências'].split(';') if not pd.isna(row['Dependências']) else [],
            'custo': row['Duração']
        }
    edges = {}
    for i, row in df.iterrows():
        for dep in nodes[row['Código']]['dependências']:
            edges[(dep, row['Código'])] = nodes[dep]['custo']
    edges_df = pd.DataFrame.from_dict(edges, orient='index', columns=['custo'])
    graph = edges_df.reset_index().rename(columns={'index': 'aresta'}).groupby(['aresta'])\
                .apply(lambda x: x.drop(columns=['aresta']).to_dict('list')).to_dict()

    print(graph)

    # print(max_path_dijkstra(graph, 's'))

 # Define uma função auxiliar para calcular o caminho máximo a partir de um nó
    def calculate_max_path(node, visited):
        if node in visited:
            return -math.inf
        visited.add(node)
        max_path = 0
        for neighbor, cost in graph.get(node, {}).items():
            max_path = max(max_path, calculate_max_path(neighbor, visited) + cost[0])
        visited.remove(node)
        return max_path

    # Encontra o caminho máximo a partir de cada nó do grafo
    max_path = -math.inf
    for node in nodes:
        max_path = max(max_path, calculate_max_path(node, set()))

    # Encontra o caminho máximo e imprime seu custo
    for node in nodes:
        visited = set()
        path = [node]
        path_cost = 0
        while True:
            max_neighbor = None
            max_cost = -math.inf
            for neighbor, cost in graph.get(node, {}).items():
                if neighbor not in visited and cost[0] > max_cost:
                    max_neighbor = neighbor
                    max_cost = cost[0]
            if max_neighbor is None:
                break
            node = max_neighbor
            visited.add(node)
            path.append(node)
            path_cost += max_cost
        if path_cost == max_path:
            print(f'Caminho máximo: {path} (custo: {path_cost})')
            break


# Função de baixo tentei implementar, mas sem sucesso

'''def max_path_dijkstra(graph, source):
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

    return path, max_dist '''

#-----------------------------------------------------------------

'''# Função para encontrar o vértice com a menor distância
def minDistance(dist, sptSet):
    min = float("inf")
    min_index = -1
    for v in range(len(dist)):
        if dist[v] < min and sptSet[v] == False:
            min = dist[v]
            min_index = v
    return min_index

# Função para imprimir o vetor de distâncias
def printSolution(dist):
    print("Vértice \tDistância da origem")
    for i in range(len(dist)):
        print(i, "\t\t", dist[i])

# Função que implementa o algoritmo de Dijkstra
def dijkstra(graph, src):
    # Inicializar o vetor de distâncias com infinito
    dist = [float("inf")] * len(graph)
    # Inicializar o vetor de predecessores com None
    pred = [None] * len(graph)
    # Inicializar o conjunto de vértices processados com False
    sptSet = [False] * len(graph)
    # Atribuir a distância da origem a zero
    dist[src] = 0
    # Repetir até que todos os vértices sejam processados
    for _ in range(len(graph)):
        # Escolher o vértice de menor distância que não foi processado
        u = minDistance(dist, sptSet)
        # Marcar esse vértice como processado
        sptSet[u] = True
        # Atualizar as distâncias dos vértices adjacentes a u
        for v in range(len(graph)):
            if graph[u][v] > 0 and sptSet[v] == False and dist[v] > dist[u] + graph[u][v]:
                dist[v] = dist[u] + graph[u][v]
                pred[v] = u
    # Imprimir o vetor de distâncias e o vetor de predecessores
    printSolution(dist)
    print("Vértice \tPredecessor")
    for i in range(len(pred)):
        print(i, "\t\t", pred[i])'''





'''import pandas as pd
import copy

def create_graph_from_csv(file_path):
    # Lê o arquivo CSV como um DataFrame do Pandas
    df = pd.read_csv(file_path)

    # Cria um dicionário com as informações de cada nó do grafo
    nodes = {}
    for i, row in df.iterrows():
        nodes[row['Código']] = {
            'dependências': row['Dependências'].split(';') if not pd.isna(row['Dependências']) else [],
            'custo': row['Duração']
        }

    # Cria um dicionário com as informações de cada aresta do grafo
    edges = {}
    for i, row in df.iterrows():
        for dep in nodes[row['Código']]['dependências']:
            edges[(dep, row['Código'])] = nodes[dep]['custo']

    # Cria um DataFrame com as informações das arestas
    edges_df = pd.DataFrame.from_dict(edges, orient='index', columns=['custo'])

    # Cria o grafo com as informações dos nós e das arestas
    graph = edges_df.reset_index().rename(columns={'index': 'aresta'}).groupby(['aresta'])\
                .apply(lambda x: x.drop(columns=['aresta']).to_dict('list')).to_dict()

    find_longest_pat(graph)

def find_longest_pat(graph):
    # Inicializa o dicionário de caminhos mais longos com caminhos de comprimento zero
    longest_paths = {node: (0, []) for node in graph}

    def dfs(node):
        # Percorre todos os vizinhos do nó atual
        for neighbor, cost in graph[node].items():
            # Calcula o comprimento do caminho mais longo para o vizinho
            path_len = longest_paths[node][0] + cost
            # Se o caminho para o vizinho for mais longo do que qualquer outro caminho encontrado,
            # atualiza o dicionário de caminhos mais longos para incluir o vizinho como próximo nó no caminho mais longo
            if path_len > longest_paths[neighbor][0]:
                longest_paths[neighbor] = (path_len, longest_paths[node][1] + [neighbor])
                # Faz uma chamada recursiva ao DFS para o vizinho encontrado
                dfs(neighbor)

    # Chama o DFS para cada nó no grafo
    for node in graph:
        dfs(node)

    # Encontra o caminho mais longo no dicionário de caminhos mais longos
    max_len = -float('inf')
    max_path = []
    for node, (length, path) in longest_paths.items():
        if length > max_len:
            max_len = length
            max_path = path

    # Imprime o caminho mais longo e seu comprimento
    print(f'Caminho mais longo: {max_path}, comprimento: {max_len}')'''
