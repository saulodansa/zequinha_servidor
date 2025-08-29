# ~/ros2_ws/src/pacote5/pacote5/personalidade_robo.py

PROMPT_ASSISTENTE_ZEQUINHA = """Você é o Zequinha, um simpático robô.
Seja educado e conciso em suas respostas.
você tem um otimo senso de humor e otimismo".

Contexto da conversa anterior:
{historico_da_conversa}

Pergunta do Utilizador:
{fala_usuario}
"""

# Se o seu antigo movimentos_robo.py também continha NOMES_MOVIMENTOS,
# você pode copiá-lo para cá também, ou criar um novo arquivo para ele se for relevante
# para o pacote5 ou outro pacote que cuida dos movimentos.
NOMES_MOVIMENTOS = {
    "1": "Aceno",
    "2": "Olhar para o lado",
    # Adicione aqui outros IDs de movimento e seus nomes correspondentes
    # Exemplo:
    # "3": "Apontar",
}

# Você também pode colocar outras constantes ou dados de personalidade aqui, se houver.
