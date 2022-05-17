import time

class Grafo:

    def __init__(self, num_vert=0, num_arestas=0, lista_adj=None, mat_adj=None, e=None):
        self.num_vert = num_vert
        self.num_arestas = num_arestas
        if lista_adj is None:
            self.lista_adj = [[] for i in range(num_vert)]
        else:
            self.lista_adj = lista_adj
        if mat_adj is None:
            self.mat_adj = [[0 for j in range(num_vert)] for i in range(num_vert)]
        else:
            self.mat_adj = mat_adj

        if e is None:
            self.e = [[] for i in range(num_vert)]
        else:
            self.e = e


    def add_aresta(self, u, v, w=1):
        """Adiciona aresta de u a v com peso w"""
        self.num_arestas += 1
        if u < self.num_vert and v < self.num_vert:
            self.lista_adj[u].append((v, w))
            self.mat_adj[u][v] = w
        else:
            print("Aresta invalida!")

    def remove_aresta(self, u, v):
        """Remove aresta de u a v, se houver"""
        if u < self.num_vert and v < self.num_vert:
            if self.mat_adj[u][v] != 0:
                self.num_arestas += 1
                self.mat_adj[u][v] = 0
                for (v2, w2) in self.lista_adj[u]:
                    if v2 == v:
                        self.lista_adj[u].remove((v2, w2))
                        break
            else:
                print("Aresta inexistente!")
        else:
            print("Aresta invalida!")

    def grau(self, u):
        """Retorna o grau do vertice u"""
        return len(self.lista_adj[u])

    def adjacente(self, u, v):
        """Determina se v é adjacente a u"""
        if self.mat_adj[u][v] != 0:
            return True
        else:
            return False

    def adjacentes_peso(self, u):
        """Retorna a lista dos vertices adjacentes a u no formato (v, w)"""
        return self.lista_adj[u]

    def adjacentes(self, u):
        """Retorna a lista dos vertices adjacentes a u"""
        adj = []
        for i in range(self.lista_adj[u]):
            (v, w) = self.lista_adj[u][i]
            adj.append(v)
        return adj

    def densidade(self):
        """Retorna a densidade do grafo"""
        return self.num_arestas / (self.num_vert * (self.num_vert - 1))

    def subgrafo(self, g2):
        """Determina se g2 e subgrafo de self"""
        if g2.num_vert > self.num_vert:
            return False
        for i in range(len(g2.mat_adj)):
            for j in range(len(g2.mat_adj[i])):
                if g2.mat_adj[i][j] != 0 and self.mat_adj[i][j] == 0:
                    return False
        return True

    def busca_largura(self, s, d):
        """Retorna a ordem de descoberta dos vertices pela
           busca em largura a partir de s"""
        # só se aplica se o grafo não for ponderado
        t = time.time()
        inf = float('inf')
        dist = [inf for v in range(self.num_vert)]
        pred = [None for v in range(self.num_vert)]
        Q = [s]
        dist[s] = 0
        while Q:
            u = Q.pop(0)
            for (v, w) in self.lista_adj[u]:
                if dist[v] == inf:  # o certo é "se desc[v] == inf"
                    Q.append(v)
                    dist[v] = dist[u] + 1
                    pred[v] = u

        C = [d]
        i = d
        while i != s:
            C.append(pred[i])
            i = pred[i]

        C.reverse()
        tf = time.time()
        print("Caminho: ", C)
        print("Custo: {}".format(dist[d]))
        print("Tempo de execução: {:.3f} s".format(tf-t))


    def dijkstra(self, s, d):
        t = time.time()
        inf = float('inf')
        dist = [inf for v in range(self.num_vert)]
        pred = [None for v in range(self.num_vert)]
        dist[s] = 0
        Q = [V for V in range(self.num_vert)]  # atrubui os vértices à Q
        while Q:  # enquanto Q diferente de 0
            u = None
            min_dist = inf
            for i in Q:
                if dist[i] < min_dist:
                    u = i
                    min_dist = dist[i]
            Q.remove(u)
            for (v, w) in self.lista_adj[u]:
                if dist[v] > dist[u] + w:
                    dist[v] = dist[u] + w
                    pred[v] = u
        C = [d]
        i = d
        while i != s:
            C.append(pred[i])
            i = pred[i]

        C.reverse()
        tf = time.time()
        print("Caminho: ", C)
        print("Custo: {}".format(dist[d]))
        print("Tempo de execução: {:.3f} s".format(tf - t))

    def bellman_ford(self, s, d):
        t = time.time()
        inf = float('inf')
        dist = [inf for v in range(self.num_vert)]
        pred = [None for v in range(self.num_vert)]
        dist[s] = 0

        for i in range(self.num_vert - 1):
            trocou = False

            for (u, v, w) in self.e:
                print("cheguei aqui")
                if dist[v] > dist[u] + w:
                    dist[v] = dist[u] + w
                    pred[v] = u
                    trocou = True

            if (trocou == False):
                break

        print(pred)
        """C = [d]
        i = d
        while i != s:
            C.append(pred[i])
            i = pred[i]

        C.reverse()
        tf = time.time()
        print("Caminho: ", C)
        print("Custo: {}".format(dist[d]))
        print("Tempo de execução: {:.3f} s".format(tf - t))"""

    def ler_arquivo(self, nome_arq):
        """Le arquivo de grafo no formato dimacs"""
        try:
            arq = open(nome_arq)
            # Leitura do cabecalho
            str = arq.readline()
            str = str.split(" ")
            self.num_vert = int(str[0])
            cont_arestas = int(str[1])
            # Inicializacao das estruturas de dados
            self.lista_adj = [[] for i in range(self.num_vert)]
            self.mat_adj = [[0 for j in range(self.num_vert)] for i in range(self.num_vert)]
            # Le cada aresta do arquivo
            for i in range(0, cont_arestas):
                str = arq.readline()
                str = str.split(" ")
                u = int(str[0])  # Vertice origem
                v = int(str[1])  # Vertice destino
                w = int(str[2])  # Peso da aresta
                self.add_aresta(u, v, w)
        except IOError:
            print("Nao foi possivel encontrar ou ler o arquivo!")
            return False
