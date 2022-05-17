import grafo
import time

arquivo = input("Informe o arquivo: ")
s = int(input("Origem: "))  # Origem
d = int(input("Destino: "))  # Destino
opcao = int(input("Digite a opção de algoritmo: \n1. Dijkstra\n2. Bellman_ford\n3.Busca Largura\n\n > "))
print("Processando...")
print("\n")
g = grafo.Grafo()
if (g.ler_arquivo(arquivo) == False):
    print("Programa encerrado.")
else:
    if opcao == 1:
        g.dijkstra(s, d)
    if opcao == 2:
        g.bellman_ford(s, d)
    if opcao == 3:
        g.busca_largura(s, d)
    if opcao > 3:
        print("Nenhum algoritmo localizado")
    if opcao < 1:
        print("Nenhum algoritmo localizado")