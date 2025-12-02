import numpy as np
import uaibot as ub
import sys
from enum import Enum
import math
import random
import os



PI = math.pi
cyl_radius = round(random.uniform(0.15, 0.3), 3)

class Direction(Enum):
    UP = 1
    DOWN = 2
    LEFT = 3
    RIGHT = 4
    NONE = 5

# Número de juntas
robot = ub.Robot.create_kuka_lbr_iiwa();
n = len(robot.links)

#Declaração do robô e do cilindro
cyl_height = 2
cilindro = ub.Cylinder(htm=ub.Utils.trn([0.8,0,0.7])*ub.Utils.rotx(PI/2), radius=cyl_radius, height = cyl_height, color="red")

# sim = ub.Simulation.create_sim_sky([robot, cilindro])
sim = ub.Simulation.create_sim_mountain([robot, cilindro])


#VARIÁVEIS DE TESTE
frame1 = frame = ub.Frame(htm=np.identity(4), size=0.3)
sim.add([frame1])

# Inicializações
hist_time = []
hist_qdot = hist_q = np.matrix(np.zeros((7, 0)))
hist_error_ori = np.matrix(np.zeros((3, 0)))
hist_r = np.matrix(np.zeros((4, 0)))
hist_u = np.matrix(np.zeros((7, 0)))

# Parâmetros de simulação
dt, t, time_max, K = 0.01, 0, 60, 1
imax = round(time_max / dt)

# Parâmetros de movimento
seg_const = round(0.01 * cyl_radius, 5)
# x_c -> profundidade do desenho
x_c, T_seg, Size_seg = 0.8-cyl_radius-0.05, 2, 0.1 + seg_const
# posição dos segmentos para y
y_1, y_2 = 0.05 + (2.0*seg_const), -0.05 - (2.0*seg_const)
# posição dos segmentos para z
z_1, z_2, z_3 = 0.8 + (6.0*seg_const), 0.7, 0.6 - (6.0*seg_const)
# posição dos segmento para x (p -> profundidade)
theta = np.arcsin(Size_seg/cyl_radius)
x_p = -(cyl_radius - np.cos(theta)*cyl_radius)
offset_board = 0.005
YD = 0.15

last_x_p = 0




# Configurações iniciais
z_inicial = np.matrix([1, 0, 0]).reshape((3, 1))
# z_d = np.matrix([1, 0, 0]).reshape((3, 1))
draw_points = np.zeros((3, 0))
reached_board = False
ind_reached = int(0.05 * imax)
s_0 = np.array([[0.1], [0.0], [0.15]])
r = np.matrix(np.zeros((4, 1)))
u = np.matrix(np.zeros((6, 1)))
painting = False

# Input do usuário
number_input = input("Digite um número de três dígitos: ").strip()

if not number_input.isdigit() or len(number_input) != 3:
    print("Entrada inválida: o número precisa ter três dígitos")
    sys.exit()

number_list = [int(dig) for dig in number_input]

# Desenho dos segmentos de reta, seguindo o display de sete segmentos
def draw_seg(t, pos, x, y, z, dir):
  m_t = t - T_seg*int(t/T_seg)
  m_z = z
  vel_seg = (Size_seg / T_seg)
  s_dot = np.matrix([0, 0, 0]).reshape((3, 1))
  match pos:
    case 0:
      m_y = y + YD
    case 1:
      m_y = y
    case 2:
      m_y = y - YD
  match dir:
    case Direction.RIGHT:
      m_y = m_y - m_t * vel_seg
      s_dot = np.matrix([0, -vel_seg, 0]).reshape((3, 1))
    case Direction.LEFT:
      m_y = m_y + m_t * vel_seg
      s_dot = np.matrix([0, vel_seg, 0]).reshape((3, 1))
    case Direction.DOWN:
      m_z = m_z - m_t * vel_seg
      s_dot = np.matrix([0, 0, -vel_seg]).reshape((3, 1))
    case Direction.UP:
      m_z = m_z + m_t * vel_seg
      s_dot = np.matrix([0, 0, vel_seg]).reshape((3, 1))
  return np.matrix([x_c - x, m_y, m_z]).reshape((3, 1)), s_dot

def calc_xp(tt, T_seg_c, z):
  global last_x_p
  if z == z_1:
    theta = -(np.arcsin((0.1*(-tt+T_seg_c)/T_seg)/cyl_radius))
  else:
    theta = np.arcsin((0.1*(tt-T_seg_c+T_seg)/T_seg)/cyl_radius)
  # z_d = np.matrix([np.cos(theta), 0, np.sin(theta)]).reshape((3, 1))
  last_x_p = -(cyl_radius - np.cos(theta)*cyl_radius)
  return -(cyl_radius - np.cos(theta)*cyl_radius)

# Desenho dos dígitos, de 0 a 9
def draw_0(tt, pos):
    global painting
    painting = False
    if tt <= 1.5*T_seg:
        return draw_seg(tt, pos, offset_board, y_2, z_1, Direction.NONE)
    elif tt <= 2*T_seg:
        return draw_seg(tt, pos, x_p, y_2, z_1, Direction.NONE)
    elif tt <= 3*T_seg:
        painting = True
        return draw_seg(tt, pos, calc_xp(tt, 3*T_seg, z_1), y_2, z_1, Direction.DOWN)
    elif tt <= 4*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 4*T_seg, z_2), y_2, z_2, Direction.DOWN)
    elif tt <= 5*T_seg:
        return draw_seg(tt, pos, x_p, y_2, z_3, Direction.LEFT) #
    elif tt <= 6*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 6*T_seg, z_1), y_1, z_3, Direction.UP)
    elif tt <= 7*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 7*T_seg, z_2), y_1, z_2, Direction.UP)
    elif tt <= 8*T_seg:
        return draw_seg(tt, pos, x_p, y_1, z_1, Direction.RIGHT) #
    else:
        painting = False
        return draw_seg(tt, pos, offset_board, y_2, z_1, Direction.NONE)

def draw_1(tt, pos):
    global painting
    if tt <= 1.5*T_seg:
        return draw_seg(tt, pos, offset_board, y_2, z_1, Direction.NONE)
    elif tt <= 2*T_seg:
        return draw_seg(tt, pos, x_p, y_2, z_1, Direction.NONE)
    elif tt <= 3*T_seg:
        painting = True
        return draw_seg(tt, pos, calc_xp(tt, 3*T_seg, z_1), y_2, z_1, Direction.DOWN)
    elif tt <= 4*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 4*T_seg, z_2), y_2, z_2, Direction.DOWN)
    else:
        painting = False
        return draw_seg(tt, pos, offset_board, y_2, z_3, Direction.NONE)

def draw_2(tt, pos):
    global painting
    if tt <= 1.5*T_seg:
        return draw_seg(tt, pos, offset_board, y_1, z_1, Direction.NONE)
    elif tt <= 2*T_seg:
        return draw_seg(tt, pos, x_p, y_1, z_1, Direction.NONE)
    elif tt <= 3*T_seg:
        painting = True
        return draw_seg(tt, pos, x_p, y_1, z_1, Direction.RIGHT)
    elif tt <= 4*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 4*T_seg, z_1), y_2, z_1, Direction.DOWN)
    elif tt <= 5*T_seg:
        return draw_seg(tt, pos, 0, y_2, z_2, Direction.LEFT)
    elif tt <= 6*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 6*T_seg, z_2), y_1, z_2, Direction.DOWN)
    elif tt <= 7*T_seg:
        return draw_seg(tt, pos, x_p, y_1, z_3, Direction.RIGHT)
    else:
        painting = False
        return draw_seg(tt, pos, offset_board, y_2, z_3, Direction.NONE)

def draw_3(tt, pos):
    global painting
    if tt <= 1.5*T_seg:
        return draw_seg(tt, pos, offset_board, y_1, z_1, Direction.NONE)
    elif tt <= 2*T_seg:
        return draw_seg(tt, pos, x_p, y_1, z_1, Direction.NONE)
    elif tt <= 3*T_seg:
        painting = True
        return draw_seg(tt, pos, x_p, y_1, z_1, Direction.RIGHT)
    elif tt <= 4*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 4*T_seg, z_1), y_2, z_1, Direction.DOWN)
    elif tt <= 5*T_seg:
        return draw_seg(tt, pos, 0, y_2, z_2, Direction.LEFT)
    elif tt <= 6*T_seg:
        return draw_seg(tt, pos, 0, y_1, z_2, Direction.RIGHT)
    elif tt <= 7*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 7*T_seg, z_2), y_2, z_2, Direction.DOWN)
    elif tt <= 8*T_seg:
        return draw_seg(tt, pos, x_p, y_2, z_3, Direction.LEFT)
    else:
        painting = False
        return draw_seg(tt, pos, offset_board, y_1, z_3, Direction.NONE)

def draw_4(tt, pos):
    global painting
    if tt <= 1.5*T_seg:
        return draw_seg(tt, pos, offset_board, y_1, z_1, Direction.NONE)
    elif tt <= 2*T_seg:
        return draw_seg(tt, pos, x_p, y_1, z_1, Direction.NONE)
    elif tt <= 3*T_seg:
        painting = True
        return draw_seg(tt, pos, calc_xp(tt, 3*T_seg, z_1), y_1, z_1, Direction.DOWN)
    elif tt <= 4*T_seg:
        return draw_seg(tt, pos, 0, y_1, z_2, Direction.RIGHT)
    elif tt <= 5*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 5*T_seg, z_2), y_2, z_2, Direction.UP)
    elif tt <= 6*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 6*T_seg, z_1), y_2, z_1, Direction.DOWN)
    elif tt <= 7*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 7*T_seg, z_2), y_2, z_2, Direction.DOWN)
    else:
        painting = False
        return draw_seg(tt, pos, offset_board, y_2, z_3, Direction.NONE)

def draw_5(tt, pos):
    global painting
    if tt <= 1.5*T_seg:
        return draw_seg(tt, pos, offset_board, y_2, z_1, Direction.NONE)
    elif tt <= 2*T_seg:
        return draw_seg(tt, pos, x_p, y_2, z_1, Direction.NONE)
    elif tt <= 3*T_seg:
        painting = True
        return draw_seg(tt, pos, x_p, y_2, z_1, Direction.LEFT)
    elif tt <= 4*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 4*T_seg, z_1), y_1, z_1, Direction.DOWN)
    elif tt <= 5*T_seg:
        return draw_seg(tt, pos, 0, y_1, z_2, Direction.RIGHT)
    elif tt <= 6*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 6*T_seg, z_2), y_2, z_2, Direction.DOWN)
    elif tt <= 7*T_seg:
        return draw_seg(tt, pos, x_p, y_2, z_3, Direction.LEFT)
    else:
        painting = False
        return draw_seg(tt, pos, offset_board, y_1, z_3, Direction.NONE)

def draw_6(tt, pos):
    global painting
    if tt <= 1.5*T_seg:
        return draw_seg(tt, pos, offset_board, y_2, z_1, Direction.NONE)
    elif tt <= 2*T_seg:
        return draw_seg(tt, pos, x_p, y_2, z_1, Direction.NONE)
    elif tt <= 3*T_seg:
        painting = True
        return draw_seg(tt, pos, x_p, y_2, z_1, Direction.LEFT)
    elif tt <= 4*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 4*T_seg, z_1), y_1, z_1, Direction.DOWN)
    elif tt <= 5*T_seg:
        return draw_seg(tt, pos, 0, y_1, z_2, Direction.RIGHT)
    elif tt <= 6*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 6*T_seg, z_2), y_2, z_2, Direction.DOWN)
    elif tt <= 7*T_seg:
        return draw_seg(tt, pos, x_p, y_2, z_3, Direction.LEFT)
    elif tt <= 8*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 8*T_seg, z_1), y_1, z_3, Direction.UP)
    else:
        painting = False
        return draw_seg(tt, pos, offset_board, y_1, z_2, Direction.NONE)

def draw_7(tt, pos):
    global painting
    if tt <= 1.5*T_seg:
        return draw_seg(tt, pos, offset_board, y_1, z_1, Direction.NONE)
    elif tt <= 2*T_seg:
        return draw_seg(tt, pos, x_p, y_1, z_1, Direction.NONE)
    elif tt <= 3*T_seg:
        painting = True
        return draw_seg(tt, pos, x_p, y_1, z_1, Direction.RIGHT)
    elif tt <= 4*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 4*T_seg, z_1), y_2, z_1, Direction.DOWN)
    elif tt <= 5*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 5*T_seg, z_2), y_2, z_2, Direction.DOWN)
    else:
        painting = False
        return draw_seg(tt, pos, offset_board, y_2, z_3, Direction.NONE)

def draw_8(tt, pos):
    global painting
    if tt <= 1.5*T_seg:
        return draw_seg(tt, pos, offset_board, y_2, z_2, Direction.NONE)
    elif tt <= 2*T_seg:
        return draw_seg(tt, pos, 0, y_2, z_2, Direction.NONE)
    elif tt <= 3*T_seg:
        painting = True
        return draw_seg(tt, pos, 0, y_2, z_2, Direction.LEFT)
    elif tt <= 4*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 4*T_seg, z_2), y_1, z_2, Direction.UP)
    elif tt <= 5*T_seg:
        return draw_seg(tt, pos, x_p, y_1, z_1, Direction.RIGHT)
    elif tt <= 6*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 6*T_seg, z_1), y_2, z_1, Direction.DOWN)
    elif tt <= 7*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 7*T_seg, z_2), y_2, z_2, Direction.DOWN)
    elif tt <= 8*T_seg:
        return draw_seg(tt, pos, x_p, y_2, z_3, Direction.LEFT)
    elif tt <= 9*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 9*T_seg, z_1), y_1, z_3, Direction.UP)
    else:
        painting = False
        return draw_seg(tt, pos, offset_board, y_1, z_2, Direction.NONE)

def draw_9(tt, pos):
    global painting
    if tt <= 1.5*T_seg:
        return draw_seg(tt, pos, offset_board, y_2, z_2, Direction.NONE)
    elif tt <= 2*T_seg:
        return draw_seg(tt, pos, 0, y_2, z_2, Direction.NONE)
    elif tt <= 3*T_seg:
        painting = True
        return draw_seg(tt, pos, 0, y_2, z_2, Direction.LEFT)
    elif tt <= 4*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 4*T_seg, z_2), y_1, z_2, Direction.UP)
    elif tt <= 5*T_seg:
        return draw_seg(tt, pos, x_p, y_1, z_1, Direction.RIGHT)
    elif tt <= 6*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 6*T_seg, z_1), y_2, z_1, Direction.DOWN)
    elif tt <= 7*T_seg:
        return draw_seg(tt, pos, calc_xp(tt, 7*T_seg, z_2), y_2, z_2, Direction.DOWN)
    elif tt <= 8*T_seg:
        return draw_seg(tt, pos, x_p, y_2, z_3, Direction.LEFT)
    else:
        painting = False
        return draw_seg(tt, pos, offset_board, y_1, z_3, Direction.NONE)

def get_num_seg(n):
    base = 3
    match n:
        case 0 | 6 | 3 | 9: return base + 6
        case 1: return base + 2
        case 2 | 5: return base + 5
        case 4: return base + 5
        case 7: return base + 3
        case 8: return base + 7

# Obtém a posição desejada do efetuador
def posicaoDesejada(t):
    num = number_list
    T1 = T_seg * get_num_seg(num[0])
    T2 = T1 + T_seg * get_num_seg(num[1])
    T3 = T2 + T_seg * get_num_seg(num[2])

    if t <= T1:
        tt = t
        return globals()[f"draw_{num[0]}"](tt, 0)
    elif t <= T2:
        tt = t - T1
        return globals()[f"draw_{num[1]}"](tt, 1)
    elif t <= T3:
        tt = t - T2
        return globals()[f"draw_{num[2]}"](tt, 2)
    else:
        return np.matrix([0, 0, 1]).reshape((3, 1)), np.matrix([0, 0, 0]).reshape((3, 1))


def normal_do_cilindro(s_d, _t):
    # Calcula a normal do cilindro para a posição desejada S_d
    z_c = cilindro.htm[0:3,-2]
    s_c = cilindro.htm[0:3,-1]

    D = s_d-s_c

    normal = D - z_c*(D.T*z_c)

    normal_unit = normal / np.linalg.norm(normal)

    return -1* normal_unit

def fun_d_cilindro(_t):

    # Posição sobre o cilindro
    s_d, s_d_dot = posicaoDesejada(_t)
    z_d = normal_do_cilindro(s_d, _t)
    z_d /= np.linalg.norm(z_d)

    return s_d, z_d, s_d_dot



def construir_htm(s_d, z_d):
    z = np.asarray(z_d).flatten()
    z = z / np.linalg.norm(z)

    # Usa o eixo do cilindro como referência para o X
    x_ref = np.asarray(cilindro.htm[0:3, 0]).flatten()

    # Remove componente de z (garante ortogonalidade)
    x = x_ref - np.dot(x_ref, z) * z
    x = x / np.linalg.norm(x)

    y = np.cross(z, x)

    R = np.column_stack((x, y, z))  # [x | y | z]
    htm = np.eye(4)
    htm[0:3, 0:3] = R
    htm[0:3, 3] = np.asarray(s_d).flatten()
    return htm

#-----------------VARIÁVEIS DE TESTE
points = []
norma_ff = []

def fun_control(_q, _t):
  global draw_points

  jac, htm = robot.jac_geo()
  jac_v = jac[0:3,:]
  jac_w = jac[3:6,:]

  #posições do efetuador
  s_e = htm[0:3, -1]
  z_e = htm[0:3, -2]

  s_d, z_d, s_d_dot =  fun_d_cilindro(_t)

  points.append(s_d)

  r = np.vstack((s_e-s_d, 1-z_d.T*z_e))

  jac_r = np.vstack((jac_v, z_d.T*ub.Utils.S(z_e)*jac_w))

  # Calcula o FeedForward apenas quando está desenhando no cilindro
  if painting:
    ff = np.block([[-s_d_dot],[0]])
  else:
    ff = 0

  K = 0.5

  r_mod = np.multiply(np.sign(r), np.sqrt(np.abs(r)))
  u = ub.Utils.dp_inv(jac_r,0.002) * (-K * r_mod - ff)

  # # O robô registra as posições ao longo da trajetória para criar um desenho
  if painting:
    draw_points = np.block([draw_points, s_e])
  else:
    draw_points = np.block([draw_points, s_0])


  return u, r

q = np.matrix(robot.q)
dt = 0.01

hist_r = []
hist_t = []
hist_z_d = []
hist_z_e = []
hist_u = []


for i in range(imax):

    t = i*dt

    if i % 50 == 0 or i == imax - 1:
        sys.stdout.write('\r')
        sys.stdout.write("[%-20s] %d%%" % ('=' * round(20 * i / (imax - 1)), round(100 * i / (imax - 1))))
        sys.stdout.flush()

    u,r = fun_control(q,t)
    q = q+u*dt

    robot.add_ani_frame(time=t, q=q)

    hist_t.append(t)
    hist_r.append(r.copy())
    hist_u.append(u.copy())

    s_e = robot.htm[0:3,-1]

    if (not reached_board) and (abs(s_e[0, 0] - cilindro.htm[0, 3]) < cilindro.height / 2 + 0.01):
        reached_board = True
        ind_reached = i

    t += dt

#-----------------VARIÁVEIS DE TESTE
# pc = ub.PointCloud(points=points, size=0.02, color='cyan')
# sim.add([pc])

# Adiciona os pontos desenhados ao quadro
point_cloud = ub.PointCloud(name="drawing", points=draw_points, size=0.02)
sim.add(point_cloud)

for i in range(imax):
    if i < ind_reached:
        point_cloud.add_ani_frame(i * dt, 0, 0)
    else:
        point_cloud.add_ani_frame(i * dt, ind_reached, i)

# Roda a simulação
output_path = "/content/animacoes/"
# sim.save(address=output_path, file_name="simulacao_2")
sim.run()