import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import threading
import logging
from openai import OpenAI
import os

class AssistenteChatGPTNode(Node):
    def __init__(self):
        super().__init__('assistente_chatgpt_node')
        self.get_logger().info('*** Nó "Cérebro" (Assistente ChatGPT) iniciado ***')

        # Inicializa cliente ChatGPT usando a variável de ambiente OPENAI_API_KEY
        self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

        # Controle de concorrência
        self.llm_response_lock = threading.Lock()
        self.llm_is_busy = False

        # Prompt do sistema e histórico
        self.system_prompt = "Você é o Zequinha, um simpático robô. Seja educado e conciso em suas respostas."
        self.history = []

        self.get_logger().info('ASSISTENTE: Prompt base e mecanismos de controle carregados.')

        # Subscription para receber a fala do usuário
        self.user_text_subscription = self.create_subscription(
            String,
            '/fala_do_usuario',
            self.llm_callback,
            10
        )
        self.get_logger().info('ASSISTENTE: Subscrito ao tópico /fala_do_usuario.')

        # Publisher para enviar a resposta ao robô
        self.robot_response_publisher = self.create_publisher(
            String,
            '/robo_zequinha/resposta_texto',
            10
        )
        self.get_logger().info('ASSISTENTE: Publicador criado para o tópico /robo_zequinha/resposta_texto.')

    def get_llm_response(self, user_input):
        # Adiciona input do usuário ao histórico
        self.history.append({"role": "user", "content": user_input})

        try:
            response = self.client.chat.completions.create(
                model="gpt-4o-mini",
                messages=[{"role": "system", "content": self.system_prompt}] + self.history,
                max_tokens=150,
                temperature=0.7
            )
            llm_response = response.choices[0].message.content.strip()

            # Adiciona resposta do assistente ao histórico
            self.history.append({"role": "assistant", "content": llm_response})
            return llm_response

        except Exception as e:
            self.get_logger().error(f"ASSISTENTE: Erro ao obter resposta do ChatGPT: {e}")
            return "Desculpe, não consegui me conectar à minha inteligência. Tente novamente."

    def llm_callback(self, msg):
        user_input = msg.data

        with self.llm_response_lock:
            if self.llm_is_busy:
                self.get_logger().warn('ASSISTENTE: Ignorando nova pergunta, ChatGPT ocupado.')
                return
            self.llm_is_busy = True

        t_start = time.time()
        llm_response_text = self.get_llm_response(user_input)
        t_end = time.time()

        # Publica a resposta
        if llm_response_text:
            response_msg = String()
            response_msg.data = llm_response_text
            self.robot_response_publisher.publish(response_msg)

        with self.llm_response_lock:
            self.llm_is_busy = False

        self.get_logger().info(f"--- FALA: '{user_input}' | RESPOSTA: '{llm_response_text}' | TEMPO: {t_end - t_start:.2f}s ---")


def main(args=None):
    rclpy.init(args=args)
    assistente_node = AssistenteChatGPTNode()
    try:
        rclpy.spin(assistente_node)
    except KeyboardInterrupt:
        pass
    finally:
        assistente_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

