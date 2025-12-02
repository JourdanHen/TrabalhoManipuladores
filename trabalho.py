### código escreve bem o 1 e o resta descaralha

import numpy as np
import uaibot as ub
import sys
import math
import random
from enum import Enum

# ==============================================================================
# 1. CONFIGURAÇÕES GERAIS E CONSTANTES
# ==============================================================================
PI = math.pi

# Configurações do Cilindro
RAIO_CILINDRO = round(random.uniform(0.15, 0.3), 3)
ALTURA_CILINDRO = 2.0
POSICAO_CILINDRO = [0.8, 0, 0.7]

# Configurações de Desenho (Especificações do Problema)
DISTANCIA_SEGURANCA = 0.05  # 50mm de distância da superfície
POSICAO_X_SUPERFICIE = 0.8 - RAIO_CILINDRO - DISTANCIA_SEGURANCA

# Configurações dos Segmentos (Tamanho e Tempo)
TAMANHO_SEGMENTO = 0.1 + round(0.01 * RAIO_CILINDRO, 5)
TEMPO_POR_SEGMENTO = 2.0  # Segundos para desenhar cada traço
ESPACO_ENTRE_DIGITOS = 0.15 # Distância Y entre os números
OFFSET_SEM_TINTA = 0.005 # Recuo extra quando não está desenhando


# Definição dos Limites do "Display de 7 Segmentos" no Espaço 3D
# Y: Horizontal, Z: Vertical
Y_SUPERIOR = 0.05 + (2.0 * round(0.01 * RAIO_CILINDRO, 5))
Y_INFERIOR = -0.05 - (2.0 * round(0.01 * RAIO_CILINDRO, 5))
Z_TOPO = 0.8 + (6.0 * round(0.01 * RAIO_CILINDRO, 5))
Z_MEIO = 0.7
Z_BASE = 0.6 - (6.0 * round(0.01 * RAIO_CILINDRO, 5))

# Estado Global de Pintura
estou_pintando = False

class Direcao(Enum):
    CIMA = 1
    BAIXO = 2
    ESQUERDA = 3
    DIREITA = 4
    NENHUMA = 5

# ==============================================================================
# 2. SETUP DO AMBIENTE (ROBÔ E SIMULAÇÃO)
# ==============================================================================
robot = ub.Robot.create_kuka_lbr_iiwa()

# Criação do Cilindro (Rotacionado para ficar deitado)
cilindro = ub.Cylinder(
    htm=ub.Utils.trn(POSICAO_CILINDRO) * ub.Utils.rotx(PI/2),
    radius=RAIO_CILINDRO,
    height=ALTURA_CILINDRO,
    color="red"
)

sim = ub.Simulation.create_sim_mountain([robot, cilindro])
sim.add([ub.Frame(htm=np.identity(4), size=0.3)]) # Frame na base

# ==============================================================================
# 3. FUNÇÕES MATEMÁTICAS AUXILIARES
# ==============================================================================

def calcular_profundidade_x(tempo_atual, tempo_inicio_segmento, z_atual):
    """
    Calcula a profundidade X necessária para manter o desenho curvo
    acompanhando a superfície do cilindro.
    """
    # Lógica simplificada: projeta o arco do cilindro no plano
    delta_t = tempo_atual
    if z_atual == Z_TOPO:
         theta = -(np.arcsin((TAMANHO_SEGMENTO * (-delta_t + tempo_inicio_segmento) / TEMPO_POR_SEGMENTO) / RAIO_CILINDRO))
    else:
         theta = np.arcsin((TAMANHO_SEGMENTO * (delta_t - tempo_inicio_segmento + TEMPO_POR_SEGMENTO) / TEMPO_POR_SEGMENTO) / RAIO_CILINDRO)

    # Retorna a posição X ajustada pela curvatura
    return -(RAIO_CILINDRO - np.cos(theta) * RAIO_CILINDRO)

def calcular_vetor_normal_cilindro(posicao_desejada):
    """
    Retorna o vetor Normal unitário à superfície do cilindro no ponto desejado.
    Essencial para que a 'caneta' do robô fique sempre perpendicular ao cilindro.
    """
    centro_cilindro = cilindro.htm[0:3, -1]
    eixo_z_cilindro = cilindro.htm[0:3, -2] # Eixo longitudinal do cilindro

    vetor_diferenca = posicao_desejada - centro_cilindro

    # Projeção ortogonal para achar a normal
    projecao = vetor_diferenca - eixo_z_cilindro * (vetor_diferenca.T * eixo_z_cilindro)
    normal_unit = projecao / np.linalg.norm(projecao)

    return -1 * normal_unit # Inverte para apontar para fora/dentro conforme referencial

# ==============================================================================
# 4. LÓGICA DE TRAJETÓRIA (DESENHO DOS NÚMEROS)
# ==============================================================================

def desenhar_segmento(t, pos_digito, x_offset, y_base, z_base, direcao):
    """
    Gera a posição (x,y,z) e a velocidade linear para um único segmento de reta.
    """
    # Tempo local dentro deste segmento (0 a T_seg)
    t_local = t - TEMPO_POR_SEGMENTO * int(t / TEMPO_POR_SEGMENTO)

    pos_y = y_base
    pos_z = z_base

    # Ajusta posição Y baseado em qual dígito estamos desenhando (1º, 2º ou 3º)
    if pos_digito == 0: pos_y += ESPACO_ENTRE_DIGITOS
    elif pos_digito == 2: pos_y -= ESPACO_ENTRE_DIGITOS

    velocidade = TAMANHO_SEGMENTO / TEMPO_POR_SEGMENTO
    vel_vector = np.matrix([0, 0, 0]).reshape((3, 1))

    # Aplica o movimento linear dependendo da direção
    if direcao == Direcao.DIREITA:
        pos_y -= t_local * velocidade
        vel_vector[1] = -velocidade
    elif direcao == Direcao.ESQUERDA:
        pos_y += t_local * velocidade
        vel_vector[1] = velocidade
    elif direcao == Direcao.BAIXO:
        pos_z -= t_local * velocidade
        vel_vector[2] = -velocidade
    elif direcao == Direcao.CIMA:
        pos_z += t_local * velocidade
        vel_vector[2] = velocidade

    # Posição final (X fixo na superfície + offset, Y calculado, Z calculado)
    pos_final = np.matrix([POSICAO_X_SUPERFICIE - x_offset, pos_y, pos_z]).reshape((3, 1))

    return pos_final, vel_vector

# --- Definição dos Dígitos (Máquina de Estados) ---
# X_P é a profundidade curva calculada. Quando é 0 ou offset, é reta.

def draw_0(tt, pos):
    global estou_pintando
    estou_pintando = False
    ts = TEMPO_POR_SEGMENTO

    # Movimento de aproximação
    if tt <= 1.5*ts: return desenhar_segmento(tt, pos, OFFSET_SEM_TINTA, Y_INFERIOR, Z_TOPO, Direcao.NENHUMA)
    elif tt <= 2*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 2*ts, Z_TOPO), Y_INFERIOR, Z_TOPO, Direcao.NENHUMA)

    # Começa a pintar
    elif tt <= 3*ts:
        estou_pintando = True
        return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 3*ts, Z_TOPO), Y_INFERIOR, Z_TOPO, Direcao.BAIXO)
    elif tt <= 4*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 4*ts, Z_MEIO), Y_INFERIOR, Z_MEIO, Direcao.BAIXO)
    elif tt <= 5*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 5*ts, Z_BASE), Y_INFERIOR, Z_BASE, Direcao.ESQUERDA)
    elif tt <= 6*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 6*ts, Z_TOPO), Y_SUPERIOR, Z_BASE, Direcao.CIMA)
    elif tt <= 7*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 7*ts, Z_MEIO), Y_SUPERIOR, Z_MEIO, Direcao.CIMA)
    elif tt <= 8*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 8*ts, Z_TOPO), Y_SUPERIOR, Z_TOPO, Direcao.DIREITA)

    # Fim
    else:
        estou_pintando = False
        return desenhar_segmento(tt, pos, OFFSET_SEM_TINTA, Y_INFERIOR, Z_TOPO, Direcao.NENHUMA)

def draw_1(tt, pos):
    global estou_pintando
    ts = TEMPO_POR_SEGMENTO
    if tt <= 1.5*ts: return desenhar_segmento(tt, pos, OFFSET_SEM_TINTA, Y_INFERIOR, Z_TOPO, Direcao.NENHUMA)
    elif tt <= 2*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 2*ts, Z_TOPO), Y_INFERIOR, Z_TOPO, Direcao.NENHUMA)
    elif tt <= 3*ts:
        estou_pintando = True
        return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 3*ts, Z_TOPO), Y_INFERIOR, Z_TOPO, Direcao.BAIXO)
    elif tt <= 4*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 4*ts, Z_MEIO), Y_INFERIOR, Z_MEIO, Direcao.BAIXO)
    else:
        estou_pintando = False
        return desenhar_segmento(tt, pos, OFFSET_SEM_TINTA, Y_INFERIOR, Z_BASE, Direcao.NENHUMA)

def draw_2(tt, pos):
    global estou_pintando
    ts = TEMPO_POR_SEGMENTO
    if tt <= 1.5*ts: return desenhar_segmento(tt, pos, OFFSET_SEM_TINTA, Y_INFERIOR, Z_TOPO, Direcao.NENHUMA)
    elif tt <= 2*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 2*ts, Z_TOPO), Y_INFERIOR, Z_TOPO, Direcao.NENHUMA)
    elif tt <= 3*ts:
        estou_pintando = True
        return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 3*ts, Z_TOPO), Y_INFERIOR, Z_TOPO, Direcao.DIREITA)
    elif tt <= 4*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 4*ts, Z_TOPO), Y_SUPERIOR, Z_TOPO, Direcao.BAIXO)
    elif tt <= 5*ts: return desenhar_segmento(tt, pos, 0, Y_SUPERIOR, Z_MEIO, Direcao.ESQUERDA) # 0 pois é reto horizontal
    elif tt <= 6*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 6*ts, Z_MEIO), Y_INFERIOR, Z_MEIO, Direcao.BAIXO)
    elif tt <= 7*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 7*ts, Z_BASE), Y_INFERIOR, Z_BASE, Direcao.DIREITA)
    else:
        estou_pintando = False
        return desenhar_segmento(tt, pos, OFFSET_SEM_TINTA, Y_SUPERIOR, Z_BASE, Direcao.NENHUMA)

def draw_3(tt, pos):
    global estou_pintando
    ts = TEMPO_POR_SEGMENTO
    if tt <= 1.5*ts: return desenhar_segmento(tt, pos, OFFSET_SEM_TINTA, Y_INFERIOR, Z_TOPO, Direcao.NENHUMA)
    elif tt <= 2*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 2*ts, Z_TOPO), Y_INFERIOR, Z_TOPO, Direcao.NENHUMA)
    elif tt <= 3*ts:
        estou_pintando = True
        return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 3*ts, Z_TOPO), Y_INFERIOR, Z_TOPO, Direcao.DIREITA)
    elif tt <= 4*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 4*ts, Z_TOPO), Y_SUPERIOR, Z_TOPO, Direcao.BAIXO)
    elif tt <= 5*ts: return desenhar_segmento(tt, pos, 0, Y_SUPERIOR, Z_MEIO, Direcao.ESQUERDA)
    elif tt <= 6*ts: return desenhar_segmento(tt, pos, 0, Y_INFERIOR, Z_MEIO, Direcao.DIREITA)
    elif tt <= 7*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 7*ts, Z_MEIO), Y_SUPERIOR, Z_MEIO, Direcao.BAIXO)
    elif tt <= 8*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 8*ts, Z_BASE), Y_SUPERIOR, Z_BASE, Direcao.ESQUERDA)
    else:
        estou_pintando = False
        return desenhar_segmento(tt, pos, OFFSET_SEM_TINTA, Y_INFERIOR, Z_BASE, Direcao.NENHUMA)

def draw_4(tt, pos):
    global estou_pintando
    ts = TEMPO_POR_SEGMENTO
    if tt <= 1.5*ts: return desenhar_segmento(tt, pos, OFFSET_SEM_TINTA, Y_INFERIOR, Z_TOPO, Direcao.NENHUMA)
    elif tt <= 2*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 2*ts, Z_TOPO), Y_INFERIOR, Z_TOPO, Direcao.NENHUMA)
    elif tt <= 3*ts:
        estou_pintando = True
        return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 3*ts, Z_TOPO), Y_INFERIOR, Z_TOPO, Direcao.BAIXO)
    elif tt <= 4*ts: return desenhar_segmento(tt, pos, 0, Y_INFERIOR, Z_MEIO, Direcao.DIREITA)
    elif tt <= 5*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 5*ts, Z_MEIO), Y_SUPERIOR, Z_MEIO, Direcao.CIMA)
    elif tt <= 6*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 6*ts, Z_TOPO), Y_SUPERIOR, Z_TOPO, Direcao.BAIXO)
    elif tt <= 7*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 7*ts, Z_MEIO), Y_SUPERIOR, Z_MEIO, Direcao.BAIXO)
    else:
        estou_pintando = False
        return desenhar_segmento(tt, pos, OFFSET_SEM_TINTA, Y_SUPERIOR, Z_BASE, Direcao.NENHUMA)

def draw_5(tt, pos):
    global estou_pintando
    ts = TEMPO_POR_SEGMENTO
    if tt <= 1.5*ts: return desenhar_segmento(tt, pos, OFFSET_SEM_TINTA, Y_SUPERIOR, Z_TOPO, Direcao.NENHUMA)
    elif tt <= 2*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 2*ts, Z_TOPO), Y_SUPERIOR, Z_TOPO, Direcao.NENHUMA)
    elif tt <= 3*ts:
        estou_pintando = True
        return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 3*ts, Z_TOPO), Y_SUPERIOR, Z_TOPO, Direcao.ESQUERDA)
    elif tt <= 4*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 4*ts, Z_TOPO), Y_INFERIOR, Z_TOPO, Direcao.BAIXO)
    elif tt <= 5*ts: return desenhar_segmento(tt, pos, 0, Y_INFERIOR, Z_MEIO, Direcao.DIREITA)
    elif tt <= 6*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 6*ts, Z_MEIO), Y_SUPERIOR, Z_MEIO, Direcao.BAIXO)
    elif tt <= 7*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 7*ts, Z_BASE), Y_SUPERIOR, Z_BASE, Direcao.ESQUERDA)
    else:
        estou_pintando = False
        return desenhar_segmento(tt, pos, OFFSET_SEM_TINTA, Y_INFERIOR, Z_BASE, Direcao.NENHUMA)

def draw_6(tt, pos):
    global estou_pintando
    ts = TEMPO_POR_SEGMENTO
    if tt <= 1.5*ts: return desenhar_segmento(tt, pos, OFFSET_SEM_TINTA, Y_SUPERIOR, Z_TOPO, Direcao.NENHUMA)
    elif tt <= 2*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 2*ts, Z_TOPO), Y_SUPERIOR, Z_TOPO, Direcao.NENHUMA)
    elif tt <= 3*ts:
        estou_pintando = True
        return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 3*ts, Z_TOPO), Y_SUPERIOR, Z_TOPO, Direcao.ESQUERDA)
    elif tt <= 4*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 4*ts, Z_TOPO), Y_INFERIOR, Z_TOPO, Direcao.BAIXO)
    elif tt <= 5*ts: return desenhar_segmento(tt, pos, 0, Y_INFERIOR, Z_MEIO, Direcao.DIREITA)
    elif tt <= 6*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 6*ts, Z_MEIO), Y_SUPERIOR, Z_MEIO, Direcao.BAIXO)
    elif tt <= 7*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 7*ts, Z_BASE), Y_SUPERIOR, Z_BASE, Direcao.ESQUERDA)
    elif tt <= 8*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 8*ts, Z_BASE), Y_INFERIOR, Z_BASE, Direcao.CIMA)
    else:
        estou_pintando = False
        return desenhar_segmento(tt, pos, OFFSET_SEM_TINTA, Y_INFERIOR, Z_MEIO, Direcao.NENHUMA)

def draw_7(tt, pos):
    global estou_pintando
    ts = TEMPO_POR_SEGMENTO
    if tt <= 1.5*ts: return desenhar_segmento(tt, pos, OFFSET_SEM_TINTA, Y_INFERIOR, Z_TOPO, Direcao.NENHUMA)
    elif tt <= 2*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 2*ts, Z_TOPO), Y_INFERIOR, Z_TOPO, Direcao.NENHUMA)
    elif tt <= 3*ts:
        estou_pintando = True
        return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 3*ts, Z_TOPO), Y_INFERIOR, Z_TOPO, Direcao.DIREITA)
    elif tt <= 4*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 4*ts, Z_TOPO), Y_SUPERIOR, Z_TOPO, Direcao.BAIXO)
    elif tt <= 5*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 5*ts, Z_MEIO), Y_SUPERIOR, Z_MEIO, Direcao.BAIXO)
    else:
        estou_pintando = False
        return desenhar_segmento(tt, pos, OFFSET_SEM_TINTA, Y_SUPERIOR, Z_BASE, Direcao.NENHUMA)

def draw_8(tt, pos):
    global estou_pintando
    ts = TEMPO_POR_SEGMENTO
    if tt <= 1.5*ts: return desenhar_segmento(tt, pos, OFFSET_SEM_TINTA, Y_SUPERIOR, Z_MEIO, Direcao.NENHUMA)
    elif tt <= 2*ts: return desenhar_segmento(tt, pos, 0, Y_SUPERIOR, Z_MEIO, Direcao.NENHUMA)
    elif tt <= 3*ts:
        estou_pintando = True
        return desenhar_segmento(tt, pos, 0, Y_SUPERIOR, Z_MEIO, Direcao.ESQUERDA)
    elif tt <= 4*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 4*ts, Z_MEIO), Y_INFERIOR, Z_MEIO, Direcao.CIMA)
    elif tt <= 5*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 5*ts, Z_TOPO), Y_INFERIOR, Z_TOPO, Direcao.DIREITA)
    elif tt <= 6*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 6*ts, Z_TOPO), Y_SUPERIOR, Z_TOPO, Direcao.BAIXO)
    elif tt <= 7*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 7*ts, Z_MEIO), Y_SUPERIOR, Z_MEIO, Direcao.BAIXO)
    elif tt <= 8*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 8*ts, Z_BASE), Y_SUPERIOR, Z_BASE, Direcao.ESQUERDA)
    elif tt <= 9*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 9*ts, Z_BASE), Y_INFERIOR, Z_BASE, Direcao.CIMA)
    else:
        estou_pintando = False
        return desenhar_segmento(tt, pos, OFFSET_SEM_TINTA, Y_INFERIOR, Z_MEIO, Direcao.NENHUMA)

def draw_9(tt, pos):
    global estou_pintando
    ts = TEMPO_POR_SEGMENTO
    if tt <= 1.5*ts: return desenhar_segmento(tt, pos, OFFSET_SEM_TINTA, Y_SUPERIOR, Z_MEIO, Direcao.NENHUMA)
    elif tt <= 2*ts: return desenhar_segmento(tt, pos, 0, Y_SUPERIOR, Z_MEIO, Direcao.NENHUMA)
    elif tt <= 3*ts:
        estou_pintando = True
        return desenhar_segmento(tt, pos, 0, Y_SUPERIOR, Z_MEIO, Direcao.ESQUERDA)
    elif tt <= 4*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 4*ts, Z_MEIO), Y_INFERIOR, Z_MEIO, Direcao.CIMA)
    elif tt <= 5*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 5*ts, Z_TOPO), Y_INFERIOR, Z_TOPO, Direcao.DIREITA)
    elif tt <= 6*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 6*ts, Z_TOPO), Y_SUPERIOR, Z_TOPO, Direcao.BAIXO)
    elif tt <= 7*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 7*ts, Z_MEIO), Y_SUPERIOR, Z_MEIO, Direcao.BAIXO)
    elif tt <= 8*ts: return desenhar_segmento(tt, pos, calcular_profundidade_x(tt, 8*ts, Z_BASE), Y_SUPERIOR, Z_BASE, Direcao.ESQUERDA)
    else:
        estou_pintando = False
        return desenhar_segmento(tt, pos, OFFSET_SEM_TINTA, Y_INFERIOR, Z_BASE, Direcao.NENHUMA)

# Mapa de complexidade (quantos segmentos cada número tem)
def obter_duracao_numero(n):
    base_inicio = 3 # Segmentos gastos para posicionar
    match n:
        case 0 | 6 | 3 | 9: return base_inicio + 6
        case 1: return base_inicio + 2
        case 2 | 5 | 4: return base_inicio + 5
        case 7: return base_inicio + 3
        case 8: return base_inicio + 7
    return base_inicio + 6

# ==============================================================================
# 5. CONTROLADOR CINEMÁTICO (CORE DO ROBÔ)
# ==============================================================================

def calcular_controle_robo(q_atual, tempo_simulacao, lista_numeros, funcoes_desenho):
    global draw_points, estou_pintando

    # 1. Obter Cinemática Direta e Jacobiano do Robô
    jacobiano_geo, htm_atual = robot.jac_geo()
    jac_velocidade = jacobiano_geo[0:3, :]
    jac_rotacao = jacobiano_geo[3:6, :]

    pos_efetuador = htm_atual[0:3, -1]
    eixo_z_efetuador = htm_atual[0:3, -2] # Eixo Z do efetuador (caneta)

    # 2. Determinar o Objetivo (Target) baseado no Tempo
    # Lógica para alternar entre o 1º, 2º e 3º número
    num1, num2, num3 = lista_numeros
    T_seg = TEMPO_POR_SEGMENTO

    T1 = T_seg * obter_duracao_numero(num1)
    T2 = T1 + T_seg * obter_duracao_numero(num2)
    T3 = T2 + T_seg * obter_duracao_numero(num3)

    # Seleciona qual função de desenho chamar
    pos_desejada = np.matrix([0,0,0]).T
    vel_desejada = np.matrix([0,0,0]).T

    if tempo_simulacao <= T1:
        # Usa reflexão ou dicionário para chamar a função correta
        pos_desejada, vel_desejada = funcoes_desenho[f"draw_{num1}"](tempo_simulacao, 0)
    elif tempo_simulacao <= T2:
        pos_desejada, vel_desejada = funcoes_desenho[f"draw_{num2}"](tempo_simulacao - T1, 1)
    elif tempo_simulacao <= T3:
        pos_desejada, vel_desejada = funcoes_desenho[f"draw_{num3}"](tempo_simulacao - T2, 2)
    else:
        # Acabou, vai para repouso
        pos_desejada = np.matrix([0, 0, 1]).reshape((3, 1))
        vel_desejada = np.matrix([0, 0, 0]).reshape((3, 1))
        estou_pintando = False

    # 3. Calcular a Orientação Desejada (Normal ao Cilindro)
    normal_desejada = calcular_vetor_normal_cilindro(pos_desejada)

    # 4. Calcular o Erro de Tarefa (Posição + Orientação)
    # Erro de posição: Onde estou - Onde deveria estar
    # Erro de orientação: 1 - produto escalar (para alinhar os vetores)
    erro_tarefa = np.vstack((pos_efetuador - pos_desejada, 1 - normal_desejada.T * eixo_z_efetuador))

    # 5. Calcular Jacobiano da Tarefa
    # Parte de orientação ajustada pela matriz antissimétrica S(z_e)
    jac_tarefa = np.vstack((jac_velocidade, normal_desejada.T * ub.Utils.S(eixo_z_efetuador) * jac_rotacao))

    # 6. Calcular Termo de FeedForward (Antecipação)
    # Compensa a velocidade do alvo para reduzir erro de rastreamento
    feed_forward = 0
    if estou_pintando:
        feed_forward = np.block([[-vel_desejada], [0]])

    # 7. Lei de Controle (Cinemática Diferencial Inversa)
    ganho_K = 0.5
    fator_amortecimento = 0.002 # Para evitar singularidades

    # Aplica raiz quadrada no erro para suavizar a aproximação (controle não-linear)
    erro_modificado = np.multiply(np.sign(erro_tarefa), np.sqrt(np.abs(erro_tarefa)))

    # u = J_inv * (-K * erro - FeedForward)
    comando_velocidade = ub.Utils.dp_inv(jac_tarefa, fator_amortecimento) * (-ganho_K * erro_modificado - feed_forward)

    # 8. Guardar pontos para visualização (Rastro)
    if estou_pintando:
        draw_points = np.block([draw_points, pos_efetuador])
    else:
        # Ponto escondido quando não pinta
        ponto_invisivel = np.array([[0.1], [0.0], [0.15]])
        draw_points = np.block([draw_points, ponto_invisivel])

    return comando_velocidade, erro_tarefa

# ==============================================================================
# 6. EXECUÇÃO PRINCIPAL
# ==============================================================================

# Input do usuário
number_input = input("Digite um número de três dígitos (000-999): ").strip()
if not number_input.isdigit() or len(number_input) != 3:
    print("Erro: Digite exatamente 3 números.")
    sys.exit()

lista_numeros = [int(dig) for dig in number_input]

# Mapeamento de funções (para evitar globals()['string'])
# IMPORTANTE: Aqui você deve conectar suas funções antigas ou refatoradas
funcoes_map = {
    "draw_0": draw_0,
    "draw_1": draw_1, "draw_2": draw_2, "draw_3": draw_3,
    "draw_4": draw_4, "draw_5": draw_5, "draw_6": draw_6,
    "draw_7": draw_7, "draw_8": draw_8, "draw_9": draw_9
}

# Inicialização de Variáveis de Simulação
draw_points = np.zeros((3, 0))
q_atual = np.matrix(robot.q)
dt = 0.01
tempo_total = 60
imax = round(tempo_total / dt)

print("Iniciando Simulação...")

for i in range(imax):
    t = i * dt

    # Barra de Progresso
    if i % 50 == 0:
        progresso = round(100 * i / imax)
        sys.stdout.write(f"\rCalculando: {progresso}% completado")
        sys.stdout.flush()

    # Loop de Controle
    velocidade_juntas, erro = calcular_controle_robo(q_atual, t, lista_numeros, funcoes_map)

    # Atualização de Euler (Integração)
    q_atual = q_atual + velocidade_juntas * dt

    # Adiciona frame na animação
    robot.add_ani_frame(time=t, q=q_atual)

# Configuração Final da Visualização (Nuvem de Pontos / Rastro)
print("\nGerando animação...")
point_cloud = ub.PointCloud(name="rastro_tinta", points=draw_points, size=0.02)
sim.add(point_cloud)

# Animação simplificada do rastro (aparece tudo no final ou progressivo)
for i in range(imax):
    point_cloud.add_ani_frame(i * dt, 0, i) # Mostra o ponto i no tempo i

# Salvar e Rodar
sim.run()