#!/bin/bash

# Carrega a chave da OpenAI do arquivo seguro em secrets, evitando que a chave fique no GitHub
export $(grep -v '^#' ~/ros2_servidor_ws/secrets/chave_da_openai.env | xargs)

# --- Configurações Iniciais ---
SERVIDOR_VENV_PATH="/home/saulo/ros2_servidor_ws/.venv/bin/python3"
VITS_PACKAGE_DIR="/home/saulo/ros2_servidor_ws/src/pacote7_vits"
VITS_IMAGE_NAME="vits_server_image"
VITS_CONTAINER_NAME="vits_server_container"

if [ ! -f "$SERVIDOR_VENV_PATH" ]; then
    echo "Erro: O ambiente Python virtual do servidor não foi encontrado."
    echo "Verifique se o caminho do seu venv está correto."
    exit 1
fi

# ---

# Passo 1: Gerenciar o Servidor VITS
echo "Gerenciando o servidor VITS..."
docker stop "$VITS_CONTAINER_NAME" > /dev/null 2>&1
docker rm "$VITS_CONTAINER_NAME" > /dev/null 2>&1

echo "  -> Verificando e construindo a imagem Docker..."
cd "$VITS_PACKAGE_DIR"
docker build -t "$VITS_IMAGE_NAME" -f docker/Dockerfile .

if [ $? -ne 0 ]; then
    echo "Erro: A construção da imagem Docker falhou. Abortando o script."
    exit 1
fi

echo "  -> Imagem Docker construída. Iniciando o container..."
docker run --rm -d -p 5002:5002 --name "$VITS_CONTAINER_NAME" "$VITS_IMAGE_NAME"
echo "  -> Esperando o servidor VITS estar pronto..."
TIMEOUT=30
for i in $(seq 1 $TIMEOUT); do
    if curl -s http://localhost:5002 > /dev/null; then
        echo "  -> Servidor VITS pronto!"
        break
    fi
    sleep 1
    echo -n "."
done

if [ $i -eq $TIMEOUT ]; then
    echo "Erro: Tempo limite de $TIMEOUT segundos excedido. O servidor VITS não iniciou."
    exit 1
fi

cd "/home/saulo/ros2_servidor_ws"

# ---

# Passo 2: Limpeza Completa e Reconstrução do ROS 2
echo "Limpando e reconstruindo o workspace..."
rm -rf build install log
colcon build
#colcon build --packages-skip pacote6_llm pacote6_llm_gemini



if [ $? -ne 0 ]; then
    echo "Erro: A compilação com colcon falhou. Abortando o script."
    exit 1
fi

# ---

# Passo 3: Correção de Executáveis e Permissões
echo "Corrigindo executáveis e permissões..."

EXECUTABLES=(
    "transcriber_node"
    "llm_server"
    "assistente_chatgpt"
    "speech_synthesis_publisher" # <<< NOME NOVO
)


for exe_name in "${EXECUTABLES[@]}"; do
    find_path=$(find install/ -name "$exe_name")
    if [ -f "$find_path" ]; then
        sed -i "1s|.*|#!$SERVIDOR_VENV_PATH|" "$find_path"
        chmod +x "$find_path"
        echo "  -> Corrigido e permissões em: $find_path"
    else
        echo "  -> AVISO: Não foi possível encontrar o arquivo: $exe_name. Verifique a compilação."
    fi
done
echo "Todos os executáveis foram processados."

# ---

# Passo 4: Fonte do Ambiente ROS 2 e Lançamento do Sistema
echo "Fonteando o ambiente e iniciando o sistema..."
source install/local_setup.bash
ros2 launch robot_bringup start_robot_chatgpt.launch.py

echo "Execução do sistema finalizada."
