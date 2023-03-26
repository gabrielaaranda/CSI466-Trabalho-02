from weightedgaph import readSubject

caminho = 1

while caminho != '0' :
    caminho = input("Informe o aquivo (0 para sair): ")
    
    if caminho != '0':
        readSubject(caminho)
