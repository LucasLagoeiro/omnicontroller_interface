import os
import signal
import subprocess

class ROS2LaunchProc:
    """Classe para gerenciar processos ROS 2 (launch/run) de forma independente."""

    def __init__(self, type: str, pkg: str, executable: str, args: dict[str, str] | None = None):
        self.type = type                      # Tipo: "launch" ou "run"
        self.pkg = pkg                        # Nome do pacote ROS 2
        self.executable = executable + ".launch.py"  # Nome do arquivo executável
        self.args = args or {}                # Argumentos extras
        self.proc: subprocess.Popen | None = None    # Processo associado

    def start(self, logger, ros2_command: str = "launch"):
        """Inicia o processo ROS 2."""
        # Verifica se já está rodando
        if self.proc and self.proc.poll() is None:
            return False

        # Se for do tipo 'run', remove a extensão de launch
        if self.type == "run":
            self.executable = self.executable.replace(".launch.py", "")

        # Verifica se há parâmetros válidos
        if self.type == "" or self.pkg == "" or self.executable == ".launch.py":
            logger.info("Nenhum arquivo de launch especificado.")
            return False

        # Monta lista de argumentos
        arglist = [f"{k}:={v}" for k, v in self.args.items()]

        # Montando o comando do ROS 2 para executar
        cmd = (
            "source /opt/ros/humble/setup.bash && "
            "source $OMNICARE_WS/install/setup.bash && "
            f"ros2 {self.type} {self.pkg} {self.executable} " + " ".join(arglist)
        )

        try:
            # Inicia subprocesso com grupo próprio (permite kill em grupo)
            self.proc = subprocess.Popen(
                ["bash", "-lc", cmd],
                preexec_fn=os.setsid,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
        except Exception as e:
            logger.info(f"Erro ao iniciar o processo: {e}")
            self.proc = None
            return False
        
        logger.info(f"Processo: {cmd}")
        logger.info(f"Processo iniciado com PID {self.proc.pid}")
        return True

    def stop(self, timeout=5):
        """Encerra o processo de forma segura (SIGINT -> SIGTERM -> SIGKILL)."""
        if not self.proc or self.proc.poll() is not None:
            return False

        os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        try:
            self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
        finally:
            self.proc = None
        return True