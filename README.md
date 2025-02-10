# Wallerson - RAITec

O projeto Wallerson do RAITec é um robô coletor de lixo projetado para automatizar a coleta de resíduos em ambientes específicos. Ele visa combinar tecnologia e sustentabilidade, utilizando sensores e mecanismos de movimentação para identificar e coletar o lixo de maneira eficiente.

# Reconhecimento de Imagem

O reconhecimento de imagem no Wallerson é estruturado em duas fases:

1. **Detecção de Lixo** : Com o auxílio da tecnologia YOLO e da câmera ESP32-CAM, o robô é capaz de identificar se o objeto capturado pela câmera é considerado lixo ou não.
2. **Análise do Tamanho e Distância** : Esta segunda etapa, ainda em desenvolvimento, busca determinar o tamanho do lixo e a distância em que ele está do robô. Para isso, há planos de incorporar uma segunda câmera, aprimorando a precisão das análises.

# Lógica de Movimento

O Wallerson utiliza o ROS (Robot Operating System) como base para sua lógica de movimento, permitindo que ele navegue em ambientes desconhecidos de forma autônoma. Com o auxílio do sensor LIDAR, o robô é capaz de mapear o ambiente, detectar obstáculos e planejar sua trajetória em tempo real, garantindo eficiência e segurança na locomoção.
