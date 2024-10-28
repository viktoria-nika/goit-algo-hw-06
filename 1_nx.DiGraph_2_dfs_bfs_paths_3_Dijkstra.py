import  networkx as nx
import matplotlib.pyplot as plt
import heapq

# Моделюємо транспортну мережу міст України за допомогою створення графу
# Створюємо граф
# Додаємо вершини(міста)
# Додаємо ребра (відстань між містами)
# Візуалізуємо граф

G = nx.DiGraph() 

G.add_node("Одеса")
G.add_node("Умань")
G.add_node("Кіровоград")
G.add_node("Львів")

G.add_edge("Одеса", "Умань", weight=276)
G.add_edge("Одеса", "Кіровоград", weight=311)
G.add_edge("Умань", "Львів", weight=713)
G.add_edge("Кіровоград", "Львів", weight=900)
#G.add_edge("Умань", "Кіровоград", weight=180)
#G.add_edge("Одеса", "Львів", weight=794)



pop  = nx.spring_layout(G)
nx.draw(G, pop, with_labels=True, node_size=700, node_color="lightblue", font_size=12, font_weight="bold")
labels = nx.get_edge_attributes(G, "weight")
nx.draw_networkx_edge_labels(G, pop, edge_labels=labels)
plt.title("Транспортна мережа міст України")
plt.show()

# Аналіз створеного графу:

num_nodes = G.number_of_nodes()
num_edges = G.number_of_edges()
degrees = dict(G.degree())

print(f" Кількість вершин: {num_nodes}")
print(f" Кількість ребер: {num_edges}")
print(f" Степені вершин: {degrees}")

# Завдання 2 алгоритми DFS і BFS

# Реалізуємо детальний пошук dfs, в глибину
# Реалізуємо найкоротший пошук bfs, тобто в ширину

def dfs_paths(G, start, end, path=[]):
    path = path + [start]
    if start == end:
        return [path]
    if start not in G:
        return []
    paths = []
    for node in G[start]:
        if node not in path:
            new_paths = dfs_paths(G, node, end, path)
            for p in new_paths:
                paths.append(p)
    return paths

def bfs_paths(G, start, end):
    queue = [(start, [start])]
    while queue:
        (node, path) = queue.pop(0)
        for next_node in set(G[node]) - set(path):
            if next_node == end:
                yield path + [next_node]
            else:
                queue.append((next_node, path + [next_node]))

# Знаходимо відстань від Одеси до Львову використовуючи алгоритми DFS і BFS
dfs_result = list(dfs_paths(G, "Одеса", "Львів"))
bfs_result = list(bfs_paths(G, "Одеса", "Львів"))

print("DFS Paths:", dfs_result)
print("BFS Paths:", bfs_result)

# Алгоритм Дейкстри

def dijkstra(G, start):
    distances = {node: float('infinity') for node in G}
    distances[start] = 0
    pq = [(0, start)]
    while pq:
        current_distance, current_node = heapq.heappop(pq)
        if current_distance > distances[current_node]:
            continue
        for neighbor, weight in G[current_node].items():
            distance = current_distance + weight['weight']
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(pq, (distance, neighbor))
    return distances

# Додамо вагу
for edge in G.edges():
    G.edges[edge[0], edge[1]]['weight'] = 1

shortest_paths = {node: dijkstra(G, node) for node in G.nodes}
print("Найменша відстань від кожного міста:", shortest_paths)
