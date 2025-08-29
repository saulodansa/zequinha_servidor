import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import requests
import os
import io
import soundfile as sf
import time
import numpy as np
from scipy import signal
from audio_common_msgs.msg import AudioData

# --- ALTERAÇÃO: Nome da classe atualizado ---
class SpeechSynthesisPublisherNode(Node):
    def __init__(self):
        # --- ALTERAÇÃO: Nome do nó no ROS 2 atualizado ---
        super().__init__('speech_synthesis_publisher')
        self.get_logger().info('Nó Publicador de Síntese de Fala iniciado. Aguardando texto do LLM...')

        self.vits_server_url = "http://localhost:5002"

        self.subscription = self.create_subscription(
            String,
            '/robo_zequinha/resposta_texto',
            self.text_response_callback,
            10
        )
        self.get_logger().info('Subscrito ao tópico /robo_zequinha/resposta_texto.')

        self.publisher_robot_status = self.create_publisher(
            Bool, '/robo_zequinha/status_fala_robo', 10
        )
        self.get_logger().info('Publicador para status de fala do robô criado.')
        
        self.audio_publisher = self.create_publisher(AudioData, '/audio_sintetizado_retorno', 10)
        self.get_logger().info('Publicador para /audio_sintetizado_retorno criado.')

        self.target_samplerate = 22050
        self.get_logger().info(f"Taxa de amostragem alvo para síntese: {self.target_samplerate} Hz")


    def _publish_speaking_status(self, is_speaking: bool):
        msg = Bool()
        msg.data = is_speaking
        self.publisher_robot_status.publish(msg)
        self.get_logger().info(f"Publicando status de fala: {is_speaking}")


    def text_response_callback(self, msg):
        text_to_synthesize = msg.data.strip()
        self.get_logger().info(f'RECEBIDO: Texto para sintetizar: "{text_to_synthesize[:min(50, len(text_to_synthesize))]}..."')

        if not text_to_synthesize:
            self.get_logger().warn('IGNORADO: Texto recebido vazio ou apenas espaços. Não sintetizando.')
            return

        audio_url = self.sintetizar_fala_vits(text_to_synthesize)

        if audio_url:
            self.get_logger().info(f'VITS OK: Áudio gerado, URL: {audio_url}. Tentando publicar...')
            self.publish_audio_from_vits_url(audio_url)
        else:
            self.get_logger().error('VITS FALHOU: Falha ao sintetizar o áudio internamente.')
            self._publish_speaking_status(False)


    def sintetizar_fala_vits(self, texto: str):
        endpoint = f"{self.vits_server_url}/sintetizar"
        headers = {"Content-Type": "application/json"}
        payload = {"texto": texto}
        self.get_logger().info(f'ENVIANDO PARA VITS: Requisição para {endpoint} com texto de {len(texto)} caracteres.')
        try:
            response = requests.post(endpoint, headers=headers, json=payload, timeout=45)
            response.raise_for_status()
            data = response.json()
            if data.get("audio_url"):
                audio_relative_url = data["audio_url"]
                full_audio_url = f"{self.vits_server_url}{audio_relative_url}"
                self.get_logger().info('VITS RESPOSTA OK: Recebido audio_url do VITS.')
                return full_audio_url
            else:
                self.get_logger().error(f"VITS RESPOSTA INVÁLIDA: 'audio_url' não encontrado na resposta: {data.get('erro', 'Resposta inesperada')}")
                return None
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f"ERRO VITS HTTP: Falha na requisição HTTP para o servidor VITS: {e}")
            return None

    def publish_audio_from_vits_url(self, audio_url: str):
        self.get_logger().info(f"BAIXANDO E PUBLICANDO: Tentando áudio de: {audio_url}")
        try:
            audio_response = requests.get(audio_url, timeout=10)
            audio_response.raise_for_status()
            audio_data_original, samplerate_original = sf.read(io.BytesIO(audio_response.content), dtype='float32')

            self.get_logger().info(f"Áudio original: Sample Rate={samplerate_original} Hz")

            audio_data_to_publish = audio_data_original
            
            if samplerate_original != self.target_samplerate:
                self.get_logger().info(f"Convertendo Sample Rate de {samplerate_original} Hz para {self.target_samplerate} Hz...")
                num_samples_resampled = int(len(audio_data_original) * (self.target_samplerate / samplerate_original))
                audio_data_to_publish = signal.resample(audio_data_original, num_samples_resampled)

            self.get_logger().info(f"Áudio pronto para publicar com Sample Rate: {self.target_samplerate} Hz")
            
            self._publish_speaking_status(True)

            if audio_data_to_publish.size > 0:
                self.get_logger().info("PUBLICANDO: Enviando dados de áudio...")
                audio_bytes = (audio_data_to_publish * 32767).astype(np.int16).tobytes()
                
                audio_msg = AudioData()
                audio_msg.data = audio_bytes
                self.audio_publisher.publish(audio_msg)
                self.get_logger().info("PUBLICAÇÃO CONCLUÍDA.")
            else:
                self.get_logger().warn("ÁUDIO VAZIO: Nenhum dado de áudio para publicar.")

        except Exception as e:
            self.get_logger().error(f"ERRO INESPERADO (publish_audio): {e}", exc_info=True)
        finally:
            self._publish_speaking_status(False)

def main(args=None):
    rclpy.init(args=args)
    # --- ALTERAÇÃO: Nome da variável e da classe instanciada ---
    speech_synthesis_publisher_node = SpeechSynthesisPublisherNode()
    try:
        rclpy.spin(speech_synthesis_publisher_node)
    except KeyboardInterrupt:
        speech_synthesis_publisher_node.get_logger().info('Nó Publicador de Síntese de Fala interrompido por Ctrl+C.')
    finally:
        speech_synthesis_publisher_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
