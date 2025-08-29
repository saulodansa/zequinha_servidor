import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from google.cloud import speech
from google.oauth2 import service_account
import numpy as np
import os
import threading
import time

CREDENTIALS_FILE = '/home/saulo/ros2_servidor_ws/secrets/double-media-384213-ed3812ca0630.json'

LANGUAGE_CODE = 'pt-BR'
TARGET_SAMPLE_RATE = 16000
SILENCE_TIMEOUT = 0.5  # segundos de silêncio para considerar fim de fala
CHUNK_DURATION = 0.05  # duração aproximada de cada chunk em segundos

class GoogleSpeechTranscriberNode(Node):
    def __init__(self):
        super().__init__('google_speech_transcriber_node')
        self.get_logger().info('Nó de Transcrição Google Cloud iniciado.')

        # Autenticação Google Cloud
        creds_path = os.path.join(os.path.dirname(__file__), CREDENTIALS_FILE)
        credentials = service_account.Credentials.from_service_account_file(creds_path)
        self.client = speech.SpeechClient(credentials=credentials)
        self.get_logger().info('Cliente Google Cloud autenticado com sucesso.')

        # Subscription de áudio
        self.audio_subscription = self.create_subscription(
            Float32MultiArray,
            'robot/audio_stream',
            self.audio_callback,
            10
        )

        # Publicador de texto final
        self.text_publisher = self.create_publisher(String, '/fala_do_usuario', 10)

        # Buffer e controle de transcrição
        self.audio_buffer = []
        self.last_chunk_time = time.time()
        self.transcribing = False

        # Thread de processamento do buffer
        self.thread = threading.Thread(target=self.process_audio_buffer, daemon=True)
        self.thread.start()

    def audio_callback(self, msg):
        audio_np = np.array(msg.data, dtype=np.float32)
        audio_int16 = (np.clip(audio_np, -1.0, 1.0) * 32767).astype(np.int16)
        self.audio_buffer.append(audio_int16.tobytes())
        self.last_chunk_time = time.time()
        #self.get_logger().info(f"Chunk recebido: {len(audio_int16)} amostras")
        #self.get_logger().info(f".")
        print('.', end='', flush=True)

    def process_audio_buffer(self):
        while rclpy.ok():
            time.sleep(CHUNK_DURATION)
            if self.audio_buffer and (time.time() - self.last_chunk_time) > SILENCE_TIMEOUT:
                self.get_logger().info(f"Fim do áudio detectado, transcrevendo {len(self.audio_buffer)} chunks")
                audio_content = b''.join(self.audio_buffer)
                self.transcribe_audio(audio_content)
                self.audio_buffer.clear()

    def transcribe_audio(self, audio_content):
        audio = speech.RecognitionAudio(content=audio_content)
        config = speech.RecognitionConfig(
            encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
            sample_rate_hertz=TARGET_SAMPLE_RATE,
            language_code=LANGUAGE_CODE
        )
        try:
            response = self.client.recognize(config=config, audio=audio)
            transcribed_text = ''.join(result.alternatives[0].transcript for result in response.results).strip()
            if transcribed_text:
                msg = String()
                msg.data = transcribed_text
                self.text_publisher.publish(msg)
                self.get_logger().info(f'Texto final publicado: "{transcribed_text}"')
            else:
                self.get_logger().info("Transcrição vazia")
        except Exception as e:
            self.get_logger().error(f'Erro na transcrição: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = GoogleSpeechTranscriberNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
