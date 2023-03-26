import pandas as pd


def build_graph(csv_file):
    graph = {}
    df = pd.read_csv(csv_file)
    for _, row in df.iterrows():
        node = row["Código"]
        dependency = row["Dependências"]
        duration = row["Duração"]
        if node not in graph:
            graph[node] = []
        if pd.notna(dependency):
            graph[node].append((dependency, duration))
            
    return graph


def bellman_ford_maxpath(graph, source):
    dist = {node: float("-inf") for node in graph}
    dist[source] = 0
    pred = {node: None for node in graph}
    for _ in range(len(graph) - 1):
        for u in graph:
            for v, w in graph[u]:
                if dist[v] < dist[u] + w:
                    dist[v] = dist[u] + w
                    pred[v] = u
    max_path = []
    node = max(dist, key=dist.get)
    while node is not None:
        max_path.append(node)
        node = pred[node]
    max_path.reverse()
    max_cost = dist[max_path[-1]]
    return max_path, max_cost


def main():
    graph = build_graph("critical_path/TOY.csv")
    max_path, max_cost = bellman_ford_maxpath(graph, "CS001")
    print("Caminho máximo:", " -> ".join(max_path))
    print("Custo do caminho máximo:", max_cost)


if __name__ == "__main__":
    main()