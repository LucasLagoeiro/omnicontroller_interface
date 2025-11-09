from typing import Dict, Callable
from geometry_msgs.msg import Twist

# Guarda o último comando de movimento
last_velocity_command = ""

def start_launcher(launchers: Dict[str, object], logger, key: str):
    """Inicia um processo do ROS2 (launch/run) associado a um botão."""
    proc = launchers.get(key)
    if not proc:
        logger.error(f"Launcher '{key}' não configurado")
        return
    try:
        started = proc.start(logger=logger)
        logger.info(f"Launch {key} STARTED" if started else f"Launch {key} já estava rodando")
    except Exception as e:
        logger.error(f"Falha ao iniciar {key}: {e}")

def stop_robot(label, last_velocity_command) -> bool:
    """Retorna True se o comando for oposto ao anterior (para parar o robô)."""
    return (label == "Right" and last_velocity_command == "Left") or \
           (label == "Left" and last_velocity_command == "Right") or \
           (label == "Up" and last_velocity_command == "Down") or \
           (label == "Down" and last_velocity_command == "Up")

def drive(node, pub_speed, logger, timer=0.15, vx=0.0, vy=0.0, wz=0.0, label: str = ""):
    """Publica uma mensagem Twist no tópico cmd_vel para movimentar o robô."""
    global last_velocity_command

    # Verifica se precisa parar o robô
    if stop_robot(label, last_velocity_command):
        pub_speed.publish(Twist())
        logger.info("Stop")
    else:
        # Cria mensagem Twist
        t = Twist()
        t.linear.x = vx
        t.linear.y = vy
        t.angular.z = wz

        pub_speed.publish(t)
        logger.info(label)
        
    last_velocity_command = label


def stop_all(launchers: Dict[str, object], pub_speed, logger):
    """Interrompe todos os launchers e para o robô."""
    for name, proc in launchers.items():
        try:
            proc.stop()
        except Exception as e:
            logger.warn(f"Falha ao parar {name}: {e}")
    pub_speed.publish(Twist())
    logger.info("Launch STOPPED")


def unknown(logger, msg: str):
    """Handler padrão para comandos desconhecidos."""
    logger.info(f"Comando desconhecido: {msg}")


def build_handlers(launchers: Dict[str, object], pub_speed, node, logger) -> Dict[str, Callable[[], None]]:
    """
    Cria um dicionário de handlers:
    - Cada botão pode acionar um launcher ou um comando de movimento.
    """
    handlers: Dict[str, Callable[[], None]] = {}

    # Cria handler para cada launcher definido
    for name in launchers.keys():
        handlers[name] = (lambda n=name: start_launcher(launchers, logger, n))

    # Handlers fixos para movimento manual
    handlers.update({
        "RIGHT":  lambda: drive(node=node, pub_speed=pub_speed, logger=logger, wz=-0.5, label="Right"),
        "LEFT":   lambda: drive(node=node, pub_speed=pub_speed, logger=logger, wz=+0.5, label="Left"),
        "UP":     lambda: drive(node=node, pub_speed=pub_speed, logger=logger, vx=+0.3, label="Up"),
        "DOWN":   lambda: drive(node=node, pub_speed=pub_speed, logger=logger, vx=-0.3, label="Down"),
        "select": lambda: stop_all(launchers, pub_speed, logger),
    })

    return handlers
