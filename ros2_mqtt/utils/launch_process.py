import os
import signal
import subprocess

class ROS2LaunchProc:
    def __init__(self, type: str, pkg: str, executable: str, args: dict[str, str] | None = None):
        self.type = type
        self.pkg = pkg
        self.executable = executable + ".launch.py"
        self.args = args or {}
        self.proc: subprocess.Popen | None = None

    def start(self,logger,ros2_command: str = "launch"):
        if self.proc and self.proc.poll() is None:
            return False  # já está rodando

        if self.type == "run":
            self.executable = self.executable.replace(".launch.py","")

        if self.type == "" or self.pkg == "" or self.executable == ".launch.py":
            logger.info("Nenhum arquivo de launch especificado.")
            return False
            
        arglist = [f"{k}:={v}" for k, v in self.args.items()]
        cmd = (
            "source /opt/ros/humble/setup.bash && "
            "source $OMNICARE_WS/install/setup.bash && "
            f"ros2 {self.type} {self.pkg} {self.executable} " + " ".join(arglist)
        )
        try:
            # start new process group para poder enviar sinal em grupo
            self.proc = subprocess.Popen(
                ["bash", "-lc", cmd],
                preexec_fn=os.setsid,              # Linux/macOS
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
        if not self.proc or self.proc.poll() is not None:
            return False
        # Envia SIGINT para todo o grupo (equivalente a Ctrl+C na CLI)
        os.killpg(os.getpgid(self.proc.pid), signal.SIGINT)
        try:
            self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            # Se não encerrou, manda SIGTERM e, por fim, SIGKILL
            os.killpg(os.getpgid(self.proc.pid), signal.SIGTERM)
            try:
                self.proc.wait(timeout=2)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(self.proc.pid), signal.SIGKILL)
        finally:
            self.proc = None
        return True

    def is_running(self):
        return self.proc is not None and self.proc.poll() is None
