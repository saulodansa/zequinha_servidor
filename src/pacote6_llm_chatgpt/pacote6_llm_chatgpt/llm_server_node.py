# lm_server_node.py

import rclpy
from rclpy.node import Node
import subprocess
import time
import os
import requests
import json
import atexit
import signal # Necessário para enviar sinais de término ao subprocesso

# --- Configurações do Servidor LLM ---
# Estes caminhos são cruciais e devem ser verificados cuidadosamente!

# Caminho completo para o executável 'llama-server' do seu llama.cpp compilado.
# Certifique-se de que o 'llama-server' realmente existe neste diretório.
# Se você o compilou em outro lugar, ajuste este caminho.
LLAMA_CPP_DIR = "/home/saulo/llama.cpp" # Verifique se este caminho está correto no seu sistema
SERVER_EXECUTABLE = os.path.join(LLAMA_CPP_DIR, "build", "bin", "llama-server")


# Caminho para o seu arquivo de modelo Gemma GGUF dentro da instalação do ROS 2.
# Esta linha garante que o nó encontre o modelo no local pós-instalação.
MODEL_PATH = os.path.join(os.path.expanduser('~/ros2_ws/install'), 'pacote5', 'share', 'pacote5', 'models', 'gemma-3-4b-it-Q4_K_M.gguf')
# A linha acima constrói o caminho: pasta atual (__file__) -> pasta pai (pacote5/) -> models/ -> seu modelo

N_GPU_LAYERS = 90  # Número de camadas da GPU para o modelo (ajuste conforme sua GPU)
PORT = "5001"      # Porta que o servidor LLM usará (consistente com llm_manager.py do ROS 2)
CONTEXT_SIZE = "4096" # Tamanho do contexto para o modelo
HOST = "127.0.0.1" # Host local (loopback)
SERVER_URL = f"http://{HOST}:{PORT}" # URL completa para o servidor LLM

# Arquivos para os logs do servidor llama-server
# Estes logs serão colocados no diretório de logs do seu workspace ROS 2, para fácil acesso.
ROS2_LOG_DIR = os.path.expanduser("~/ros2_ws/log")
if not os.path.exists(ROS2_LOG_DIR):
    os.makedirs(ROS2_LOG_DIR) # Cria o diretório se não existir

SERVER_LOG_OUT_FILE = os.path.join(ROS2_LOG_DIR, "llama_server_stdout.log")
SERVER_LOG_ERR_FILE = os.path.join(ROS2_LOG_DIR, "llama_server_stderr.log")


class LLMServerNode(Node):
    def __init__(self):
        super().__init__('llm_server_node')
        self.get_logger().info('Nó do Servidor LLM (Gemma 3) iniciado. Preparando para lançar llama-server.')

        self.server_process = None
        self.server_stdout_log_fd = None # File descriptor para stdout
        self.server_stderr_log_fd = None # File descriptor para stderr

        # Tenta iniciar o servidor llama-server quando este nó ROS 2 é iniciado
        if not self.start_llm_server_process():
            self.get_logger().fatal("Não foi possível iniciar o processo do llama-server. Por favor, verifique os caminhos e a compilação do llama.cpp.")
            # Não chamamos rclpy.shutdown() aqui diretamente para permitir que o main() lide com isso
            # mas o erro fatal indica que o nó não será funcional.

        # Registra uma função para garantir que o processo do servidor seja parado
        # quando o nó ROS 2 for encerrado (ex: Ctrl+C ou ros2 shutdown).
        atexit.register(self.stop_llm_server_process)
        
        self.get_logger().info('Nó do Servidor LLM pronto.')


    def start_llm_server_process(self):
        """
        Inicia o processo externo do llama-server.
        Retorna True se o servidor foi iniciado ou já está rodando, False caso contrário.
        """
        # Verifica se o processo já está rodando
        if self.server_process is not None and self.server_process.poll() is None:
            self.get_logger().info(f"llama-server já parece estar rodando com PID: {self.server_process.pid}.")
            return True

        # Comando para iniciar o llama-server
        command = [
            SERVER_EXECUTABLE,
            "-m", MODEL_PATH,
            "-ngl", str(N_GPU_LAYERS),
            "--port", PORT,
            "-c", str(CONTEXT_SIZE),
            "--host", HOST,
        ]
        
        try:
            self.get_logger().info(f"Abrindo arquivos de log do servidor: {SERVER_LOG_OUT_FILE}, {SERVER_LOG_ERR_FILE}")
            # Abre os arquivos de log para redirecionar a saída do subprocesso.
            # Usamos 'with open' para garantir que os arquivos sejam fechados, mas Popen precisa de file descriptors.
            # Então, abrimos e passamos o fd, fechando-os no stop_llm_server_process.
            self.server_stdout_log_fd = open(SERVER_LOG_OUT_FILE, 'wb')
            self.server_stderr_log_fd = open(SERVER_LOG_ERR_FILE, 'wb')

            self.get_logger().info(f"Iniciando llama-server com o comando: {' '.join(command)}")
            # Inicia o processo do llama-server.
            # 'preexec_fn=os.setsid' é importante para criar um novo grupo de processo.
            # Isso nos permite encerrar o grupo de processos inteiro, incluindo o servidor e seus filhos,
            # o que é mais robusto para processos em segundo plano.
            self.server_process = subprocess.Popen(
                command,
                stdout=self.server_stdout_log_fd,
                stderr=self.server_stderr_log_fd,
                preexec_fn=os.setsid
            )
            
            self.get_logger().info(f"llama-server iniciado com PID: {self.server_process.pid}.")
            self.get_logger().info(f"Logs do servidor estão em: {SERVER_LOG_OUT_FILE} e {SERVER_LOG_ERR_FILE}")
            
            self.get_logger().info(f"Aguardando o llama-server carregar o modelo (até 60 segundos)...")
            
            # Loop para verificar se o servidor está realmente pronto para receber requisições.
            # Tenta se conectar ao endpoint padrão "/" do servidor.
            timeout_start = time.time()
            timeout_seconds = 60 # Tempo máximo para esperar o servidor iniciar
            
            while time.time() - timeout_start < timeout_seconds:
                try:
                    requests.get(f"{SERVER_URL}/", timeout=1) 
                    self.get_logger().info(f"llama-server respondendo na porta {PORT}. Pronto para uso!")
                    return True
                except requests.exceptions.ConnectionError:
                    self.get_logger().debug("Aguardando llama-server iniciar...")
                    time.sleep(2) 
                except Exception as e:
                    self.get_logger().warn(f"Erro inesperado ao verificar o llama-server: {e}. Tentando novamente.")
                    time.sleep(2)
            
            # Se saiu do loop, o servidor não respondeu dentro do tempo limite.
            self.get_logger().fatal(f"Erro CRÍTICO: O llama-server não respondeu na porta {PORT} após {timeout_seconds} segundos. Verifique os logs ({SERVER_LOG_OUT_FILE}, {SERVER_LOG_ERR_FILE}) e a instalação do llama.cpp.")
            self.stop_llm_server_process() # Tenta parar o processo se não iniciou corretamente.
            return False

        except FileNotFoundError:
            self.get_logger().fatal(f"Erro CRÍTICO: Executável do servidor não encontrado em {SERVER_EXECUTABLE}. Certifique-se de que o llama.cpp foi compilado e o caminho está correto.")
            return False
        except Exception as e:
            self.get_logger().fatal(f"Ocorreu um erro inesperado ao tentar iniciar o llama-server: {e}", exc_info=True)
            return False

    def stop_llm_server_process(self):
        """
        Para o processo externo do llama-server e fecha os arquivos de log.
        """
        if self.server_process and self.server_process.poll() is None:
            self.get_logger().info("Parando llama-server...")
            try:
                # Envia um sinal SIGTERM para o grupo de processos.
                # Isso é crucial devido ao 'os.setsid' usado no Popen para garantir que todos os filhos sejam encerrados.
                os.killpg(os.getpgid(self.server_process.pid), signal.SIGTERM)
                self.server_process.wait(timeout=10) # Espera o processo terminar suavemente
                self.get_logger().info("llama-server terminado de forma suave.")
            except subprocess.TimeoutExpired:
                self.get_logger().warn("Servidor não terminou em 10 segundos, forçando o término (SIGKILL)...")
                # Se não terminar suavemente, força o término com SIGKILL.
                os.killpg(os.getpgid(self.server_process.pid), signal.SIGKILL)
                self.server_process.wait()
                self.get_logger().info("llama-server forçado a terminar.")
            except Exception as e:
                self.get_logger().error(f"Erro ao tentar parar o llama-server: {e}", exc_info=True)
        else:
            self.get_logger().info("llama-server não estava rodando ou já foi parado.")
        
        # Fecha os file descriptors dos logs para liberar os arquivos.
        if self.server_stdout_log_fd and not self.server_stdout_log_fd.closed:
            self.server_stdout_log_fd.close()
            self.server_stdout_log_fd = None
        if self.server_stderr_log_fd and not self.server_stderr_log_fd.closed:
            self.server_stderr_log_fd.close()
            self.server_stderr_log_fd = None
        
        self.server_process = None # Reseta a referência ao processo


    def destroy_node(self):
        """
        Método chamado quando o nó ROS 2 é encerrado.
        Garante que o processo do servidor LLM seja parado e os recursos liberados.
        """
        self.get_logger().info('Chamando destroy_node do LLMServerNode...')
        self.stop_llm_server_process() # Garante que o servidor seja parado ao destruir o nó
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    llm_server_node = LLMServerNode()
    try:
        rclpy.spin(llm_server_node) # Mantém o nó ativo e processando callbacks (se houver)
    except KeyboardInterrupt:
        llm_server_node.get_logger().info("Interrupção de teclado recebida. Encerrando nó...")
    except Exception as e:
        llm_server_node.get_logger().error(f"Erro inesperado no loop principal do ROS: {e}", exc_info=True)
    finally:
        llm_server_node.destroy_node() # Garante que a função de limpeza seja chamada
        if rclpy.ok(): # Verifica se o ROS ainda está inicializado antes de desligar
            rclpy.shutdown()

if __name__ == '__main__':
    main()
