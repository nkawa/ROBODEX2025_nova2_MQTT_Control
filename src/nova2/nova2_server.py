# 一つのプロセスでNova2に接続するために使用
# ここでのみNova2Robotをインスタンス化
import multiprocessing as mp
import queue
import logging
from typing import Dict, Any
from nova2.nova2_robot import Nova2Robot

class Nova2RobotServer:
    def __init__(self, robot_ip: str = "192.168.5.1"):
        self.robot_ip = robot_ip
        self.command_queue = mp.Queue()
        self.stop_event = mp.Event()
        
    def run_proc(self, log_queue):
        logger = logging.getLogger("ROBOT-SERVER")
        if log_queue is not None:
            handler = logging.handlers.QueueHandler(log_queue)
        else:
            handler = logging.StreamHandler()
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
        
        # initializing
        logger.info("Initializing robot...")
        self.robot = Nova2Robot(host=self.robot_ip, logger=logger)
        self.robot.start()
        self.robot.clear_error()
        logger.info("Robot initialized")
        
        # メインループ
        while not self.stop_event.is_set():
            try:
                cmd = self.command_queue.get(timeout=0.001)
                self._handle_command(cmd, logger)
            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"Error in server loop: {e}")
        
        logger.info("Robot server stopped")
    

    def _handle_command(self, cmd: Dict[str, Any], logger):
        client_id = cmd['client_id']
        method = cmd['method']
        args = cmd.get('args', [])
        kwargs = cmd.get('kwargs', {})
        response_queue = cmd['response_queue']
        
        try:
            result = getattr(self.robot, method)(*args, **kwargs)
            response_queue.put({
                'status': 'success',
                'result': result
            })
        except Exception as e:
            logger.error(f"Error executing {method}: {e}")
            response_queue.put({
                'status': 'error',
                'error': str(e),
                'error_type': type(e).__name__
            })
    
    def stop(self):
        self.stop_event.set()