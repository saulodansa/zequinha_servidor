# Dockerfile para o Servidor VITS (pacote7_vits)

# Usa uma imagem base com Python e suporte a CUDA
FROM nvidia/cuda:12.1.1-cudnn8-runtime-ubuntu22.04

# Instala as dependências de sistema
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.10 python3.10-venv python3-pip \
    build-essential \
    libsndfile1 \
    ffmpeg \
    libportaudio2 \
    portaudio19-dev \
    python3.10-dev \
    libcairo2-dev pkg-config \
    libgirepository1.0-dev gobject-introspection \
    alsa-utils \
    espeak \
    && rm -rf /var/lib/apt/lists/*

# Define o diretório de trabalho principal dentro do container.
WORKDIR /app

# --- Copia os arquivos do projeto para o container ---
# Copia o conteúdo do pacote7_vits, que está no diretório de contexto.
COPY . /app/pacote7_vits
# Copia a pasta 'monotonic_align'.
COPY docker/monotonic_align /app/monotonic_align_source
# Copia o 'requirements.txt'.
COPY requirements.txt /app/requirements.txt
# ----------------------------------------------------

# Cria e ativa o ambiente virtual VITS dentro do container.
RUN python3 -m venv /app/venv_vits
ENV PATH="/app/venv_vits/bin:$PATH"

# Instala as dependências Python no ambiente virtual VITS.
RUN pip install --no-cache-dir Cython && \
    pip install --no-cache-dir \
    torch torchaudio torchvision --index-url https://download.pytorch.org/whl/cu121 \
    && pip install --no-cache-dir -r /app/requirements.txt \
    && pip install --no-cache-dir matplotlib scikit-learn librosa

# --- Compila Monotonic Align DENTRO do Container ---
RUN cd /app/monotonic_align_source && \
    python3 setup.py build_ext --inplace && \
    mkdir -p /app/monotonic_align/monotonic_align && \
    cp monotonic_align/core.*.so /app/monotonic_align/monotonic_align/ && \
    rm -rf /app/monotonic_align_source
# --- FIM: Compilação de Monotonic Align ---

# Garante que o diretório temporário para arquivos de áudio VITS possa ser escrito.
RUN mkdir -p /app/pacote7_vits/temp_audio_vits

# Expõe a porta que o servidor Flask do VITS vai usar.
EXPOSE 5002

# Define o comando que será executado quando o container iniciar.

CMD ["bash", "-c", "export PYTHONPATH=/app/pacote7_vits:$PYTHONPATH:/app && python3 /app/pacote7_vits/pacote7_vits/_vits_flask_runner.py"]
