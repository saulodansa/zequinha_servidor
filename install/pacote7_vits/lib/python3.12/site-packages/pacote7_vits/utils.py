import matplotlib
# Tenta usar um backend não interativo para evitar problemas em servidores sem GUI
try:
    matplotlib.use("Agg")
except ImportError:
    # Ignora se 'Agg' não estiver disponível (pode acontecer em alguns ambientes mínimos)
    pass
import matplotlib.pylab as plt # Importa plt globalmente
import os
import glob
import sys
import argparse
import logging
import json
import subprocess
import numpy as np
from scipy.io.wavfile import read
import torch
import librosa
# Removidas importações diretas de librosa.util, usamos librosa.* diretamente se necessário

# Flag global não é mais estritamente necessária com imports globais, mas mantida por segurança
# MATPLOTLIB_FLAG = False # Comentado/Removido

# Configuração básica de log (pode ser sobrescrita pelo train.py)
logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)
logger = logging

MAX_WAV_VALUE = 32768.0


def dynamic_range_compression_torch(x, C=1, clip_val=1e-5):
    """
    PARAMS
    ------
    C: compression factor
    """
    return torch.log(torch.clamp(x, min=clip_val) * C)


def dynamic_range_decompression_torch(x, C=1):
    """
    PARAMS
    ------
    C: compression factor used to compress
    """
    return torch.exp(x) / C


def spectral_normalize_torch(magnitudes):
    output = dynamic_range_compression_torch(magnitudes)
    return output


def spectral_de_normalize_torch(magnitudes):
    output = dynamic_range_decompression_torch(magnitudes)
    return output

# Funções de Checkpoint (parecem OK)
def load_checkpoint(checkpoint_path, model, optimizer=None):
  assert os.path.isfile(checkpoint_path)
  checkpoint_dict = torch.load(checkpoint_path, map_location='cpu')
  iteration = checkpoint_dict['iteration']
  learning_rate = checkpoint_dict['learning_rate']
  if optimizer is not None and 'optimizer' in checkpoint_dict:
    optimizer.load_state_dict(checkpoint_dict['optimizer'])
  else:
      logger.warn("Optimizer state not found or not loaded from checkpoint.")
  saved_state_dict = checkpoint_dict['model']
  if hasattr(model, 'module'):
    state_dict = model.module.state_dict()
  else:
    state_dict = model.state_dict()
  new_state_dict= {}
  for k, v in state_dict.items():
    try:
      # Tratamento para diferentes nomes de chaves (ex: DDP)
      k_in_ckpt = k
      if not k_in_ckpt in saved_state_dict and k.startswith('module.'):
           k_in_ckpt = k[len('module.'):]
      if not k_in_ckpt in saved_state_dict and not k.startswith('module.'):
           k_in_ckpt = f"module.{k}"

      if k_in_ckpt in saved_state_dict:
        new_state_dict[k] = saved_state_dict[k_in_ckpt]
      else:
        logger.info("%s is not in the checkpoint" % k)
        new_state_dict[k] = v
    except Exception as e:
      logger.info(f"Could not load param {k} from checkpoint: {e}")
      new_state_dict[k] = v
  if hasattr(model, 'module'):
    model.module.load_state_dict(new_state_dict, strict=False) # Use strict=False para mais flexibilidade
  else:
    model.load_state_dict(new_state_dict, strict=False) # Use strict=False para mais flexibilidade
  logger.info("Loaded checkpoint '{}' (iteration {})" .format(
    checkpoint_path, iteration))
  # Retorna também o estado do scaler se existir no checkpoint
  scaler_state = checkpoint_dict.get('scaler')
  return model, optimizer, learning_rate, iteration, scaler_state


def save_checkpoint(model, optimizer, learning_rate, iteration, checkpoint_path, scaler=None):
  logger.info("Saving model and optimizer state at iteration {} to {}".format(
    iteration, checkpoint_path))
  if hasattr(model, 'module'):
    state_dict = model.module.state_dict()
  else:
    state_dict = model.state_dict()
  save_dict = {'model': state_dict,
               'iteration': iteration,
               'optimizer': optimizer.state_dict(),
               'learning_rate': learning_rate}
  # Salva estado do scaler se fornecido
  if scaler is not None:
    save_dict['scaler'] = scaler.state_dict()

  torch.save(save_dict, checkpoint_path)


def summarize(writer, global_step, scalars={}, histograms={}, images={}, audios={}, audio_sampling_rate=22050):
  for k, v in scalars.items():
    writer.add_scalar(k, v, global_step)
  for k, v in histograms.items():
    writer.add_histogram(k, v, global_step)
  for k, v in images.items():
    writer.add_image(k, v, global_step, dataformats='HWC') # Espera HxWxC
  for k, v in audios.items():
    writer.add_audio(k, v, global_step, audio_sampling_rate)


def latest_checkpoint_path(dir_path, regex="G_*.pth"):
  f_list = glob.glob(os.path.join(dir_path, regex))
  if not f_list: # Retorna None se não encontrar arquivos
      return None
  f_list.sort(key=lambda f: int("".join(filter(str.isdigit, os.path.basename(f))))) # Ordena pelo número no nome do arquivo
  x = f_list[-1]
  print(f"Latest checkpoint found: {x}")
  return x

# Função de Plotagem de Espectrograma CORRIGIDA
def plot_spectrogram_to_numpy(spectrogram):
  # global MATPLOTLIB_FLAG # Não mais necessário com imports no topo
  # if not MATPLOTLIB_FLAG:
  #   # ... (código antigo de setup matplotlib) ...
  #   MATPLOTLIB_FLAG = True
  #   mpl_logger = logging.getLogger('matplotlib')
  #   mpl_logger.setLevel(logging.WARNING)

  fig, ax = plt.subplots(figsize=(10,2))
  im = ax.imshow(spectrogram, aspect="auto", origin="lower",
                  interpolation='none')
  plt.colorbar(im, ax=ax)
  plt.xlabel("Frames")
  plt.ylabel("Channels")
  plt.tight_layout()

  fig.canvas.draw()
  # --- LÓGICA CORRIGIDA ---
  buf = fig.canvas.tostring_argb()
  ncols, nrows = fig.canvas.get_width_height()
  data = np.frombuffer(buf, dtype=np.uint8).reshape(nrows, ncols, 4)[:,:,:3] # Pega RGB de ARGB/RGBA
  # --- FIM DA CORREÇÃO ---
  plt.close(fig)
  return data

# Função de Plotagem de Alinhamento CORRIGIDA
def plot_alignment_to_numpy(alignment, info=None):
  # global MATPLOTLIB_FLAG # Não mais necessário
  # if not MATPLOTLIB_FLAG:
  #   # ... (código antigo de setup matplotlib) ...
  #   MATPLOTLIB_FLAG = True
  #   mpl_logger = logging.getLogger('matplotlib')
  #   mpl_logger.setLevel(logging.WARNING)

  fig, ax = plt.subplots(figsize=(6, 4))
  im = ax.imshow(alignment.T, aspect='auto', origin='lower', # Corrigido: .T para transpor alinhamento para visualização comum
                  interpolation='none')
  fig.colorbar(im, ax=ax)
  xlabel = 'Decoder timestep'
  if info is not None:
      xlabel += '\n\n' + info
  plt.xlabel(xlabel)
  plt.ylabel('Encoder timestep')
  plt.tight_layout()

  fig.canvas.draw()
  # --- LÓGICA CORRIGIDA ---
  buf = fig.canvas.tostring_argb()
  ncols, nrows = fig.canvas.get_width_height()
  data = np.frombuffer(buf, dtype=np.uint8).reshape(nrows, ncols, 4)[:,:,:3] # Pega RGB de ARGB/RGBA
  # --- FIM DA CORREÇÃO ---
  plt.close(fig)
  return data

# Funções de carregamento de áudio (parecem OK)
def load_wav_to_torch(full_path):
  # Usa soundfile para mais robustez com formatos diferentes
  try:
      data, sampling_rate = librosa.load(full_path, sr=None) # Carrega na taxa original
  except Exception as e:
      logger.error(f"Erro ao carregar wav com librosa: {full_path} - {e}")
      # Fallback para scipy? Ou apenas retornar erro?
      # sampling_rate, data = read(full_path) # scipy pode ser menos robusto
      raise e # Propaga o erro

  # Normaliza para float32 entre -1 e 1 (se não estiver já)
  if data.dtype != np.float32:
      data = data.astype(np.float32) / np.iinfo(data.dtype).max if np.issubdtype(data.dtype, np.integer) else data.astype(np.float32)

  # Garante que é mono (pegando apenas o primeiro canal se for estéreo)
  if data.ndim > 1:
      data = data[:, 0]

  return torch.FloatTensor(data), sampling_rate

# Função load_librosa_to_torch removida pois load_wav_to_torch agora usa librosa
def load_librosa_to_torch(full_path, sampling_rate):
  data, _ = librosa.load(full_path, sr=sampling_rate)
  return torch.FloatTensor(data.astype(np.float32)), sampling_rate


# Função de carregar listas (parece OK)
def load_filepaths_and_text(filename, split="|"):
  try:
    with open(filename, encoding='utf-8') as f:
      # Adiciona verificação para pular linhas vazias
      filepaths_and_text = [line.strip().split(split) for line in f if line.strip()]
  except FileNotFoundError:
      logger.error(f"Arquivo de lista não encontrado: {filename}")
      return [] # Retorna lista vazia em caso de erro
  return filepaths_and_text

# Funções de HParams (parecem OK, mas adicionando checagem de arquivo config)
def get_hparams(init=True):
  parser = argparse.ArgumentParser()
  parser.add_argument('-c', '--config', type=str, default="./configs/base_0_speakers.json",
                      help='JSON file for configuration')
  parser.add_argument('-m', '--model', type=str, required=False, default="custom_model",
                      help='Model name')

  args = parser.parse_args()
  model_dir = os.path.join("./logs", args.model)

  if not os.path.exists(model_dir):
    os.makedirs(model_dir)

  config_path = args.config
  # Verifica se o arquivo de config existe ANTES de tentar ler
  if not os.path.isfile(config_path):
      logger.error(f"Arquivo de configuração não encontrado: {config_path}")
      sys.exit(1) # Sai se o config não existe

  config_save_path = os.path.join(model_dir, "config.json")
  if init:
    with open(config_path, "r", encoding='utf-8') as f: # Adiciona encoding
      data = f.read()
    with open(config_save_path, "w", encoding='utf-8') as f: # Adiciona encoding
      f.write(data)
  else:
    # Se não for init, tenta carregar do diretório do modelo, se existir
    if os.path.isfile(config_save_path):
        config_path = config_save_path # Usa o config salvo no log
        logger.info(f"Carregando configuração do diretório do modelo: {config_path}")
    elif not os.path.isfile(config_path): # Se nem o salvo nem o original existem
        logger.error(f"Arquivo de configuração não encontrado nem em {config_path} nem em {config_save_path}")
        sys.exit(1)
    else: # Usa o config original se o salvo não existir
        logger.info(f"Configuração salva não encontrada em {config_save_path}, usando: {config_path}")

    with open(config_path, "r", encoding='utf-8') as f:
        data = f.read()

  try:
      config = json.loads(data)
  except json.JSONDecodeError as e:
      logger.error(f"Erro ao decodificar JSON de configuração em {config_path}: {e}")
      sys.exit(1)

  hparams = HParams(**config)
  hparams.model_dir = model_dir
  return hparams


def get_hparams_from_dir(model_dir):
  config_save_path = os.path.join(model_dir, "config.json")
  if not os.path.isfile(config_save_path):
      logger.error(f"Arquivo de configuração não encontrado no diretório do modelo: {config_save_path}")
      sys.exit(1)
  with open(config_save_path, "r", encoding='utf-8') as f:
    data = f.read()
  try:
      config = json.loads(data)
  except json.JSONDecodeError as e:
      logger.error(f"Erro ao decodificar JSON de configuração em {config_save_path}: {e}")
      sys.exit(1)

  hparams =HParams(**config)
  hparams.model_dir = model_dir
  return hparams


def get_hparams_from_file(config_path):
  if not os.path.isfile(config_path):
      logger.error(f"Arquivo de configuração não encontrado: {config_path}")
      sys.exit(1)
  with open(config_path, "r", encoding='utf-8') as f:
    data = f.read()
  try:
    config = json.loads(data)
  except json.JSONDecodeError as e:
      logger.error(f"Erro ao decodificar JSON de configuração em {config_path}: {e}")
      sys.exit(1)

  hparams =HParams(**config)
  return hparams

# Função check_git_hash (parece OK)
def check_git_hash(model_dir):
  source_dir = os.path.dirname(os.path.realpath(__file__))
  git_dir = os.path.join(source_dir, ".git")
  if not os.path.isdir(git_dir): # Checa se .git é diretório
    logger.warning("{} is not a git repository, therefore hash value comparison will be ignored.".format(
      source_dir
    ))
    return

  try:
    # Usa Popen para melhor captura de erro/saída
    process = subprocess.Popen(["git", "rev-parse", "HEAD"], stdout=subprocess.PIPE, stderr=subprocess.PIPE, cwd=source_dir, text=True)
    stdout, stderr = process.communicate()
    if process.returncode != 0:
        logger.warning(f"Falha ao obter git hash: {stderr}")
        return
    cur_hash = stdout.strip()
  except FileNotFoundError:
    logger.warning("Comando 'git' não encontrado. Ignorando comparação de hash.")
    return

  path = os.path.join(model_dir, "githash")
  if os.path.exists(path):
    try:
        with open(path, 'r') as f:
            saved_hash = f.read().strip()
        if saved_hash != cur_hash:
          logger.warning("git hash values are different. {}(saved) != {}(current)".format(
            saved_hash[:8], cur_hash[:8]))
    except Exception as e:
        logger.warning(f"Falha ao ler git hash salvo: {e}")
  else:
    try:
        with open(path, "w") as f:
            f.write(cur_hash)
    except Exception as e:
        logger.warning(f"Falha ao salvar git hash atual: {e}")


# Função get_logger (parece OK)
def get_logger(model_dir, filename="train.log"):
  global logger
  logger = logging.getLogger(os.path.basename(model_dir))
  logger.setLevel(logging.DEBUG) # Nível DEBUG aqui pode ser ajustado se necessário

  # Evita adicionar handlers duplicados
  if not logger.hasHandlers():
      formatter = logging.Formatter("%(asctime)s\t%(name)s\t%(levelname)s\t%(message)s")
      if not os.path.exists(model_dir):
        os.makedirs(model_dir)

      # Handler para arquivo
      fh = logging.FileHandler(os.path.join(model_dir, filename))
      fh.setLevel(logging.DEBUG)
      fh.setFormatter(formatter)
      logger.addHandler(fh)

      # Handler para console (para ver INFO e acima no terminal)
      # sh = logging.StreamHandler(sys.stdout)
      # sh.setLevel(logging.INFO)
      # sh.setFormatter(formatter)
      # logger.addHandler(sh)

  return logger

# Classe HParams (parece OK)
class HParams():
  def __init__(self, **kwargs):
    for k, v in kwargs.items():
      if type(v) == dict:
        v = HParams(**v)
      self[k] = v

  def keys(self):
    return self.__dict__.keys()

  def items(self):
    return self.__dict__.items()

  def values(self):
    return self.__dict__.values()

  def __len__(self):
    return len(self.__dict__)

  def __getitem__(self, key):
    return getattr(self, key)

  def __setitem__(self, key, value):
    return setattr(self, key, value)

  def __contains__(self, key):
    return key in self.__dict__

  def __repr__(self):
    return self.__dict__.__repr__()