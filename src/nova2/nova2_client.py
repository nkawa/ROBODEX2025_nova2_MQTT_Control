# nova2_robot_client.py (新規ファイル)
import multiprocessing as mp
import uuid

class Nova2RobotClient:    
    def __init__(self, command_queue, client_name: str = None):
        self.command_queue = command_queue
        self.client_id = client_name or str(uuid.uuid4())
        self.response_queue = mp.Queue()
    
    def _call_remote(self, method: str, *args, **kwargs):
        self.command_queue.put({
            'client_id': self.client_id,
            'method': method,
            'args': args,
            'kwargs': kwargs,
            'response_queue': self.response_queue
        })
        
        response = self.response_queue.get(timeout=5)
        
        if response['status'] == 'error':
            error_msg = response['error']
            raise Exception(f"Remote error: {error_msg}")
        
        return response['result']
    
    # # 既存のメソッドをラップ
    # def get_current_joint(self):
    #     return self._call_remote('get_current_joint')
    
    # def get_current_pose(self):
    #     return self._call_remote('get_current_pose')
    
    # def move_joint_servo(self, joints):
    #     return self._call_remote('move_joint_servo', joints)
    
    # def is_enabled(self):
    #     return self._call_remote('is_enabled')
    
    # def enable_robot(self, ext_speed):
    #     return self._call_remote('enable_robot', ext_speed=ext_speed)
    
    # def disable(self):
    #     return self._call_remote('disable')
    
    # def clear_error(self):
    #     return self._call_remote('clear_error')
    
    # def enter_servo_mode(self):
    #     return self._call_remote('enter_servo_mode')
    
    # def leave_servo_mode(self):
    #     return self._call_remote('leave_servo_mode')
    
    # def is_in_servo_mode(self):
    #     return self._call_remote('is_in_servo_mode')
    
    # 未定義のものは動的に解決
    def __getattr__(self, name):
        def method(*args, **kwargs):
            return self._call_remote(name, *args, **kwargs)
        return method