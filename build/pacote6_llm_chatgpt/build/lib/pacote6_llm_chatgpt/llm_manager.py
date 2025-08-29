# llm_manager.py


import requests
import logging

# Endereço do seu servidor LLM. Ajuste se for diferente.
LLM_SERVER_URL = "http://127.0.0.1:5001" 

def start_llm_server():
    """
    Verifica se o servidor LLM está no ar.
    Em uma arquitetura ROS, o ideal é que o servidor seja um processo separado.
    Esta função apenas confirma se ele está acessível.
    """
    logging.basicConfig(level=logging.INFO)
    try:
        requests.get(f"{LLM_SERVER_URL}/", timeout=2)
        logging.info(f"LLM Server já está respondendo em {LLM_SERVER_URL}.")
        return True
    except requests.ConnectionError:
        logging.error(f"LLM Server não está respondendo em {LLM_SERVER_URL}. Por favor, inicie-o manualmente.")
        return False
    except Exception as e:
        logging.error(f"Erro ao verificar o LLM Server: {e}")
        return False



def get_llm_response(prompt_text: str) -> str:
    """
    Envia um prompt para o servidor LLM e retorna a resposta.
    """
    if not prompt_text:
        return "Desculpe, não entendi a pergunta."

    headers = {"Content-Type": "application/json"}
    
    # Payload ajustado para a API OpenAI-compatible do llama-server
    payload = {
        "model": "gemma-3-4b-it", # Nome do modelo que o llama-server está usando
        "messages": [{"role": "user", "content": prompt_text}],
        "max_tokens": 256, # Você pode ajustar este valor
        "temperature": 0.7, # Você pode ajustar este valor
        "stream": False # Para obter a resposta completa de uma vez
    }
    
    try:
        # Endpoint correto para o llama-server
        response = requests.post(f"{LLM_SERVER_URL}/v1/chat/completions", headers=headers, json=payload, timeout=120)
        response.raise_for_status() # Lança um HTTPError para respostas de erro (4xx ou 5xx)
        
        data = response.json()
        
        # Extrai o conteúdo da resposta, que está em 'choices[0].message.content'
        content = None
        if "choices" in data and len(data["choices"]) > 0:
            message = data["choices"][0].get("message", {})
            content = message.get("content")
        
        if content:
            return content.strip()
        else:
            logging.error("Não foi possível encontrar o campo 'content' na resposta JSON esperada do LLM server.")
            logging.debug(f"Resposta completa: {json.dumps(data, indent=2)}") # Útil para depuração
            return "Erro ao extrair conteúdo da resposta do servidor."

    except requests.exceptions.RequestException as e:
        logging.error(f"Erro ao comunicar com o LLM Server: {e}")
        return "Desculpe, estou com problemas para me conectar ao meu cérebro agora."
    except json.JSONDecodeError:
        logging.error(f"Erro ao decodificar JSON da resposta do LLM Server. Conteúdo da resposta: {response.text}")
        return "Erro ao decodificar JSON do servidor."
    except Exception as e:
        logging.error(f"Erro inesperado em get_llm_response: {e}", exc_info=True)
        return "Erro inesperado ao processar resposta do LLM."
