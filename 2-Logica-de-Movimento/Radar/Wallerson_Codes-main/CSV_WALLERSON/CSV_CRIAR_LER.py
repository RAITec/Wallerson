import csv

def salvar_dados_em_csv(arquivo_csv, dados):
    """
    Salva os dados (ângulo, distância) em um arquivo CSV.
    :param arquivo_csv: Nome do arquivo CSV.
    :param dados: Lista de tuplas (ângulo, distância).
    """
    with open(arquivo_csv, mode='w', newline='') as file:
        escritor = csv.writer(file)
        escritor.writerow(['angulo', 'distancia'])  # Cabeçalho
        escritor.writerows(dados)  # Escreve os dados linha por linha

def ler_dados_de_csv(arquivo_csv):
    """
    Lê os dados de um arquivo CSV e retorna uma lista de tuplas (ângulo, distância).
    :param arquivo_csv: Nome do arquivo CSV.
    :return: Lista de tuplas (ângulo, distância).
    """
    dados = []
    with open(arquivo_csv, mode='r') as file:
        leitor = csv.reader(file)
        next(leitor)  # Pula o cabeçalho
        for linha in leitor:
            angulo, distancia = map(float, linha)  # Converte os valores para float
            dados.append((angulo, distancia))
    return dados