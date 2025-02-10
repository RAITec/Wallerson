# Dados simulados
dados_simulados = [
    (0, 150), (10, 145), (20, 140), (30, 135), (40, 100), (50, 90), 
    (60, 120), (70, 150), (80, 170), (90, 100), (100, 170), 
    (110, 150), (120, 120), (130, 90), (140, 100), (150, 135), 
    (160, 140), (170, 145), (180, 150)
]

# Salvar os dados no CSV
arquivo_csv = 'dados_sensor.csv'
salvar_dados_em_csv(arquivo_csv, dados_simulados)
print(f"Dados salvos no arquivo {arquivo_csv}!")

# Ler os dados do CSV
dados_lidos = ler_dados_de_csv(arquivo_csv)
print(f"Dados lidos do CSV: {dados_lidos}")

# Gerar gráficos
gerar_graficos(dados_lidos)

