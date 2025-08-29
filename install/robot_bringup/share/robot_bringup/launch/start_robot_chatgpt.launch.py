# start_robot_chatgpt.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Defina o caminho para o workspace do servidor
    servidor_workspace_path = os.path.expanduser('~/ros2_servidor_ws')
    
    # Caminho do Python do ambiente virtual para o workspace do servidor
    servidor_venv_python = os.path.join(servidor_workspace_path, '.venv', 'bin', 'python3')
    
    # --- CAMINHOS PARA OS SCRIPTS DOS NÓS ---
    transcriber_script = os.path.join(servidor_workspace_path, 'src', 'pacote5_transcritor', 'pacote5_transcritor', 'google_transcriber_node.py')
    
    assistente_script = os.path.join(servidor_workspace_path, 'src', 'pacote6_llm_chatgpt', 'pacote6_llm_chatgpt', 'assistente_chatgpt_node.py')
    
    # --- ALTERAÇÃO: Adicionamos o caminho para o nosso NOVO nó publisher ---
    synthesis_publisher_script = os.path.join(servidor_workspace_path, 'src', 'pacote8_publisher_voz', 'pacote8_publisher_voz', 'speech_synthesis_publisher_node.py')

    # --- (Opcional) Mantemos o caminho antigo para referência ---
    # synthesis_script_antigo = os.path.join(servidor_workspace_path, 'src', 'pacote8_sinteseVoz', 'pacote8_sinteseVoz', 'speech_synthesis_node.py')
    
    return LaunchDescription([
        # Nó de Transcrição (Servidor)
        Node(
            package='pacote5_transcritor',
            executable=servidor_venv_python,
            arguments=[transcriber_script],
            name='google_transcriber_node',
            output='screen',
            prefix='stdbuf -o L',
        ),

        # Nó Assistente (Servidor)
        Node(
            package='pacote6_llm_chatgpt',
            executable=servidor_venv_python,
            arguments=[assistente_script],
            name='assistente_chatgpt_node',
            output='screen',
        ),
        
        Node(
            package='pacote8_publisher_voz',
            executable=servidor_venv_python,
            arguments=[synthesis_publisher_script],
            name='speech_synthesis_publisher', # Nome do nó atualizado
            output='screen',
            # vvv ADICIONE ESTA SEÇÃO DE REMAPPING ABAIXO vvv
            remappings=[
                ('entrada_texto', '/robo_zequinha/resposta_texto'),
                ('saida_audio', '/audio_sintetizado_retorno')
            ]
        ),
    ])

