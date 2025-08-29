# ~/ros2_ws/src/pacote7_vits/pacote7_vits/_vits_flask_runner.py
# Este script é o servidor Flask para VITS, executado em um subprocesso Docker.

import os
import sys
import time
import logging
import json
import re

# Imports do Flask para o servidor HTTP
from flask import Flask, request, jsonify, send_file

# Imports do VITS (PyTorch, Numpy, Soundfile)
import torch
import numpy as np
import soundfile as sf

# --- INÍCIO DA SOLUÇÃO DEFINITIVA PARA MONOTONIC_ALIGN (VIA ctypes) ---
# ESTE BLOCO ESTÁ COMENTADO TEMPORARIAMENTE PARA TESTES!
# ... (o código comentado de monotonic_align não precisa de alteração) ...
# --- FIM DA SOLUÇÃO DEFINITIVA PARA MONOTONIC_ALIGN (VIA ctypes) ---


# Seus módulos locais VITS (importações absolutas dentro do container)
# CORREÇÃO: O nome do pacote foi alterado para 'pacote7_vits'
from pacote7_vits import commons
from pacote7_vits import utils
from pacote7_vits.models import SynthesizerTrn
from pacote7_vits.text.symbols import symbols
from pacote7_vits.text import text_to_sequence, _clean_text

# --- Configurações e Constantes ---
LOG_LEVEL = logging.INFO
logging.basicConfig(level=LOG_LEVEL, format='[%(levelname)s] %(asctime)s VITS_FLASK_RUNNER: %(message)s')

# Defina o BASE_DIR_VITS para onde seus módulos e modelos foram copiados no Dockerfile
# Dentro do container, eles estão em /app/pacote7_vits
BASE_DIR_VITS_MODULES_AND_MODELS = "/app/pacote7_vits/pacote7_vits"

# Caminhos para os modelos baseados na localização dos módulos e modelos dentro do container
CONFIG_VITS_PATH = os.path.join(BASE_DIR_VITS_MODULES_AND_MODELS, "models", "base_0_speakers.json")
CHECKPOINT_VITS_PATH = os.path.join(BASE_DIR_VITS_MODULES_AND_MODELS, "models", "G_119000.pth")

# Mude esta linha para usar a nova variável de caminho base
TEMP_AUDIO_DIR = os.path.join(BASE_DIR_VITS_MODULES_AND_MODELS, "temp_audio_vits/")
if not os.path.exists(TEMP_AUDIO_DIR):
    os.makedirs(TEMP_AUDIO_DIR, exist_ok=True)
# Mude esta linha para usar a nova variável de caminho base
TEMP_AUDIO_DIR = os.path.join(BASE_DIR_VITS_MODULES_AND_MODELS, "temp_audio_vits/")
if not os.path.exists(TEMP_AUDIO_DIR):
    os.makedirs(TEMP_AUDIO_DIR, exist_ok=True)
    
_PUNCTUATION_NO_SPACE = ';:,.!?¡¿—…"«»“”'
SYMBOLS_TO_IGNORE_STRICT = set(['_'] + list(_PUNCTUATION_NO_SPACE) + ['ˈ', 'ˌ', ' '])
VOWELS_SET = set(['a', 'e', 'i', 'o', 'u', 'ɐ', 'ə', 'ɛ', 'ɔ', 'æ', 'ʊ', 'ɪ', 'y', 'ɐ̃', 'ẽ', 'ĩ', 'õ', 'ũ'])

net_g_vits = None
hps_vits = None

# --- Funções Auxiliares do VITS ---
# OBS: A função `maximum_path` (do monotonic_align) NÃO está mais definida aqui.
# A chamada a ela dentro de calculate_vits_symbol_timings será REMOVIDA ou tratada.

def get_text_for_vits(text_input, hps_config_vits):
    if hps_config_vits is None:
        logging.error("HPS (configurações do modelo VITS) não carregado em get_text_for_vits.")
        return None, None
    try:
        text_norm = _clean_text(text_input, hps_config_vits.data.text_cleaners)
        if not text_norm:
            logging.error(f"Limpeza de texto '{text_input}' resultou em string vazia.")
            return None, None
        
        sequence = text_to_sequence(text_norm, [])
        
        if sequence is None or not sequence:
            logging.error(f"text_to_sequence falhou para a string limpa: '{text_norm}'")
            return None, None

        symbols_for_alignment_calc = list(text_norm)

        if hps_config_vits.data.add_blank:
            id_sequence_for_vits = commons.intersperse(sequence, 0)
            blank_char = symbols[0]
            symbols_for_alignment_calc = commons.intersperse(symbols_for_alignment_calc, blank_char)
        else:
            id_sequence_for_vits = sequence

        tensor_ids = torch.LongTensor(id_sequence_for_vits)
        return tensor_ids, symbols_for_alignment_calc
    except Exception as e:
        logging.error(f"Erro CRÍTICO em get_text_for_vits: {e}", exc_info=True)
        return None, None

def calculate_vits_symbol_timings(alignment_path, symbols_list_for_timing, hps_config_vits):
    # Esta função ainda espera um alignment_path, mas não podemos calculá-lo sem monotonic_align
    # O VITS Server geralmente retorna [None, []] se não puder calcular timings
    logging.warning("Função calculate_vits_symbol_timings chamada, mas monotonic_align está desativado. Retornando timings vazios.")
    return [] # Sempre retorna timings vazios se monotonic_align não for usado

def carregar_modelo_vits_on_server_start_internal():
    """
    Função interna para carregar o modelo VITS.
    Adaptação da sua função original para ser chamada pelo Flask.
    """
    global net_g_vits, hps_vits
    if net_g_vits is not None and hps_vits is not None:
        logging.info("Modelo VITS já carregado anteriormente.")
        return True
    try:
        if not os.path.exists(CONFIG_VITS_PATH):
            logging.critical(f"Arquivo de configuração VITS não encontrado: {CONFIG_VITS_PATH}")
            return False
        if not os.path.exists(CHECKPOINT_VITS_PATH):
            logging.critical(f"Arquivo de checkpoint VITS não encontrado: {CHECKPOINT_VITS_PATH}")
            return False

        logging.info(f"Carregando VITS: config='{CONFIG_VITS_PATH}', checkpoint='{CHECKPOINT_VITS_PATH}'")
        hps_vits = utils.get_hparams_from_file(CONFIG_VITS_PATH)
        
        logging.info("Inicializando modelo VITS SynthesizerTrn...")
        net_g_vits = SynthesizerTrn(
            len(symbols),
            hps_vits.data.filter_length // 2 + 1,
            hps_vits.train.segment_size // hps_vits.data.hop_length,
            **hps_vits.model
        )

        if torch.cuda.is_available():
            logging.info("CUDA disponível. Movendo modelo VITS para GPU.")
            net_g_vits = net_g_vits.cuda()
        else:
            logging.info("CUDA não disponível. Usando CPU para VITS.")

        _ = net_g_vits.eval()
        logging.info(f"Carregando checkpoint VITS de: {CHECKPOINT_VITS_PATH}")
        _ = utils.load_checkpoint(CHECKPOINT_VITS_PATH, net_g_vits, None)
        
        logging.info("Modelo VITS carregado com sucesso no servidor.")
        return True
    except Exception as e:
        logging.critical(f"Erro CRÍTICO ao carregar modelo VITS: {e}", exc_info=True)
        net_g_vits = None
        hps_vits = None
        return False

def vits_gerar_audio_e_timings(texto_para_sintetizar: str):
    global net_g_vits, hps_vits

    if net_g_vits is None or hps_vits is None:
        logging.error("Modelo VITS não está carregado ou disponível para síntese.")
        return None, []

    if not texto_para_sintetizar or not texto_para_sintetizar.strip():
        logging.info("Texto vazio recebido para síntese.")
        return None, []

    audio_numpy = None
    action_timings_final = [] # Sempre vazia agora que monotonic_align está desativado
    
    timestamp = int(time.time() * 1000)
    temp_wav_filename = f"vits_output_{timestamp}.wav"
    temp_wav_filepath = os.path.join(TEMP_AUDIO_DIR, temp_wav_filename)

    try:
        stn_tst_tensor, symbols_for_alignment = get_text_for_vits(texto_para_sintetizar, hps_vits)
        if stn_tst_tensor is None or symbols_for_alignment is None:
            logging.error("Falha no pré-processamento do texto para VITS.")
            return None, []

        with torch.no_grad():
            device = "cuda" if torch.cuda.is_available() else "cpu"
            x_tst = stn_tst_tensor.unsqueeze(0).to(device)
            x_tst_lengths = torch.LongTensor([stn_tst_tensor.size(0)]).to(device)
            
            logging.info(f"Iniciando inferência VITS para: '{texto_para_sintetizar[:30].replace(os.linesep, ' ')}...'")
            inference_output = net_g_vits.infer(
                x_tst,
                x_tst_lengths,
                noise_scale=.667,
                noise_scale_w=0.8,
                length_scale=1.1
            )
            logging.info("Inferência VITS concluída.")
            
            audio_tensor_for_playback, attn_matrix, y_mask = None, None, None
            
            if isinstance(inference_output, (list, tuple)) and len(inference_output) >= 1:
                audio_tensor_for_playback = inference_output[0]
                if len(inference_output) >= 2: attn_matrix = inference_output[1]
                if len(inference_output) >= 3: y_mask = inference_output[2]
            elif torch.is_tensor(inference_output):
                audio_tensor_for_playback = inference_output
            else:
                logging.error(f"Saída da inferência VITS inesperada: {type(inference_output)}")
                return None, []

            if audio_tensor_for_playback is not None and torch.is_tensor(audio_tensor_for_playback):
                if audio_tensor_for_playback.ndim == 3 and audio_tensor_for_playback.shape[0] == 1 and audio_tensor_for_playback.shape[1] == 1:
                    audio_numpy = audio_tensor_for_playback.squeeze().data.cpu().float().numpy()
                elif audio_tensor_for_playback.ndim == 2 and audio_tensor_for_playback.shape[0] == 1:
                    audio_numpy = audio_tensor_for_playback.squeeze(0).data.cpu().float().numpy()
                elif audio_tensor_for_playback.ndim == 1:
                    audio_numpy = audio_tensor_for_playback.data.cpu().float().numpy()
                else:
                    logging.error(f"Tensor de áudio com dimensões inesperadas: {audio_tensor_for_playback.shape}")
                    audio_numpy = None
            
            if audio_numpy is None:
                logging.error("Falha ao converter tensor de áudio para numpy array.")
                return None, []

            # raw_symbol_timings = [] # Removido pois monotonic_align está desativado
            # if attn_matrix is not None and y_mask is not None and torch.is_tensor(attn_matrix) and torch.is_tensor(y_mask):
            #   logging.info("Calculando alinhamento e timings dos símbolos...")
            #   # ... Código de cálculo de timings que usava monotonic_align ...
            #   # Esse bloco inteiro será pulado.

            # Sempre retorna action_timings_final vazia se monotonic_align não for usado
            # A linha abaixo precisa ser consistente com o retorno de vits_gerar_audio_e_timings
            # e a chamada em endpoint_sintetizar
            
            # Não é necessário recalcular timings se monotonic_align está desativado
            # if raw_symbol_timings: (este if foi removido)

            # Este bloco é o que estava causando problemas, vamos comentar ou simplificar
            # if raw_symbol_timings:
            #   intermediate_symbol_timings = []
            #   for timing_data in raw_symbol_timings:
            #   if timing_data["symbol"] not in SYMBOLS_TO_IGNORE_STRICT:
            #     intermediate_symbol_timings.append(timing_data)
                    
            #   for timing_data_filt in intermediate_symbol_timings:
            #   symbol_char_to_check = timing_data_filt["symbol"]
            #   if symbol_char_to_check.lower() in VOWELS_SET:
            #     action_timings_final.append(
            #         (timing_data_filt["start_time"], timing_data_filt["end_time"], timing_data_filt["symbol"])
            #     )
            #   logging.info(f"Timings de ação calculados: {len(action_timings_final)} vogais.")
            # else:
            #   logging.warning("Matriz de atenção (attn_matrix) ou y_mask não disponível/válida. Pulando cálculo de timings detalhados.")
            
            audio_valido_para_playback = False
            if audio_numpy is not None:
                if np.isnan(audio_numpy).any() or np.isinf(audio_numpy).any():
                    logging.error("Áudio gerado contém NaN ou Inf.")
                elif np.max(np.abs(audio_numpy)) < 1e-5:
                    logging.warning("Áudio gerado é quase silêncio (amplitude muito baixa).")
                    audio_valido_para_playback = True # Ainda pode ser válido, mas é um aviso
                else:
                    audio_valido_para_playback = True
            
            if audio_valido_para_playback:
                sf.write(temp_wav_filepath, audio_numpy.astype(np.float32), hps_vits.data.sampling_rate)
                logging.info(f"Áudio salvo com sucesso em: {temp_wav_filepath}")
                return temp_wav_filename, action_timings_final # Retorna timings vazios
            else:
                logging.error("Áudio gerado não é válido para playback ou é None após processamento.")
                return None, []
                
    except Exception as e_synth_geral:
        logging.error(f"Erro GERAL em vits_gerar_audio_e_timings: {e_synth_geral}", exc_info=True)
        return None, []

# --- Configuração do Servidor Flask (Instância global) ---
app = Flask(__name__)

@app.route('/sintetizar', methods=['POST'])
def endpoint_sintetizar():
    if not request.is_json:
        return jsonify({"erro": "Requisição precisa ser JSON"}), 400
        
    data = request.get_json()
    texto = data.get('texto')

    if not texto:
        return jsonify({"erro": "Parâmetro 'texto' não fornecido"}), 400

    logging.info(f"Requisição HTTP recebida para sintetizar: '{texto[:50].replace(os.linesep, ' ')}...'")
    
    nome_arquivo_wav, timings_acao = vits_gerar_audio_e_timings(texto)

    if nome_arquivo_wav:
        return jsonify({
            "arquivo_wav": nome_arquivo_wav,
            "action_timings": timings_acao,
            "audio_url": f"/audio/{nome_arquivo_wav}",
            "mensagem": "Áudio sintetizado com sucesso."
        }), 200
    else:
        return jsonify({"erro": "Falha ao sintetizar o áudio internamente."}), 500

@app.route('/audio/<path:nome_arquivo>')
def servir_audio(nome_arquivo):
    if ".." in nome_arquivo or nome_arquivo.startswith("/"):
        logging.warning(f"Tentativa de acesso a arquivo inválido: {nome_arquivo}")
        return jsonify({"erro": "Nome de arquivo inválido"}), 400

    caminho_completo_audio = os.path.join(TEMP_AUDIO_DIR, nome_arquivo)
    
    if not os.path.exists(caminho_completo_audio) or not os.path.isfile(caminho_completo_audio):
        logging.warning(f"Arquivo de áudio não encontrado: {caminho_completo_audio}")
        return jsonify({"erro": "Arquivo de áudio não encontrado"}), 404
    
    logging.info(f"Servindo arquivo de áudio: {caminho_completo_audio}")
    try:
        return send_file(caminho_completo_audio, mimetype='audio/wav')
    except Exception as e:
        logging.error(f"Erro ao servir arquivo {caminho_completo_audio}: {e}", exc_info=True)
        return jsonify({"erro": "Erro interno ao servir o arquivo de áudio"}), 500

if __name__ == '__main__':
    # carregar_modelo_vits_on_server_start_internal()
    modelo_carregado_ok = carregar_modelo_vits_on_server_start_internal()

    if modelo_carregado_ok:
        logging.info("Iniciando servidor Flask para VITS na porta 5002...")
        app.run(host='0.0.0.0', port=5002, debug=False, use_reloader=False)
    else:
        logging.critical("Servidor VITS não pode iniciar devido a falha no carregamento do modelo. Verifique os logs.")
