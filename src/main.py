import datetime
import logging
import json
import logging.handlers
import queue
import threading
import time
import tkinter as tk
import multiprocessing
from tkinter import scrolledtext
from typing import Optional

from nova2.nova2_mqtt_control import ProcessManager

from nova2.nova2_client import Nova2RobotClient
from nova2.nova2_server import Nova2RobotServer

# 使わないけど、とりあえずインポートしておく
from nova2.tools import tool_infos
tool_ids = [tool_info["id"] for tool_info in tool_infos]



class SetAreaPopup(tk.Toplevel):
    def __init__(self, parent):
        super().__init__(parent)
        self.result = None
        self.transient(parent)
        self.grab_set()
        self.title("Set Area")
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        tk.Label(self, text="Set Area:").pack(pady=10)
        btn_frame = tk.Frame(self)
        btn_frame.pack(pady=5)
        btn_true = tk.Button(btn_frame, text="Enable", width=10,
                             command=lambda: self.button_pressed(True))
        btn_true.pack(side=tk.LEFT, padx=5)
        btn_false = tk.Button(btn_frame, text="Disable", width=10,
                              command=lambda: self.button_pressed(False))
        btn_false.pack(side=tk.LEFT, padx=5)
        btn_cancel = tk.Button(btn_frame, text="Cancel", width=10,
                               command=self.on_close)
        btn_cancel.pack(side=tk.LEFT, padx=5)

        self.wait_window()

    def button_pressed(self, value):
        self.result = value
        self.destroy()

    def on_close(self):
        self.result = None
        self.destroy()


class GUILoggingHandler(logging.Handler):
    def __init__(self, gui: "MQTTWin") -> None:
        super().__init__()
        self.gui = gui
        self._closed = False
    
    def emit(self, record: logging.LogRecord) -> None:
        try:
            msg = self.format(record)
            self.gui.gui_log_queue.put(msg, block=True, timeout=None)
        except Exception:
            self.handleError(record)

    def close(self) -> None:
        # 並列処理時のログハンドラの多重closeを防ぐ
        if not self._closed:
            super().close()
            self._closed = True


class MicrosecondFormatter(logging.Formatter):
    def formatTime(self, record, datefmt=None):
        dt = datetime.datetime.fromtimestamp(record.created)
        if datefmt:
            s = dt.strftime(datefmt)
            # %f をマイクロ秒で置換
            s = s.replace('%f', f"{dt.microsecond:06d}")
            return s
        else:
            return super().formatTime(record, datefmt)


class MQTTWin:
    def __init__(self, root, use_joint_monitor_plot: bool = False):
        self.use_joint_monitor_plot = use_joint_monitor_plot
        self.pm = ProcessManager()
        log_queue = self.pm.log_queue
        self.logging_dir = self.get_logging_dir()
        self.setup_logging(log_queue=log_queue, logging_dir=self.logging_dir)
        self.setup_logger(log_queue=log_queue)
        self.gui_log_queue = queue.Queue()
        self.logger.info("Starting Process!")
 
        self.root = root
        self.root.title("Nova2 Controller")
        self.root.geometry("1100x950")

        for col in range(10):
            self.root.grid_columnconfigure(col, weight=1, uniform="equal")
        
        row = 0
        self.button = {}
        self.button["ConnectRobot"] = tk.Button(self.root, text="ConnectRobot", padx=5,
                      command=self.ConnectRobot, state="normal")
        self.button["ConnectRobot"].grid(row=row,column=0,padx=2,pady=2,sticky="ew", columnspan=2)

        self.button["ConnectMQTT"] = tk.Button(self.root, text="ConnectMQTT", padx=5,
                             command=self.ConnectMQTT, state="normal")
        self.button["ConnectMQTT"].grid(row=row,column=2,padx=2,pady=2,sticky="ew", columnspan=2)

        self.button["DemoPutDownBox"] = tk.Button(self.root, text="no task", padx=5,
                       command=None, state="disabled")
        self.button["DemoPutDownBox"].grid(row=row,column=4,padx=2,pady=2,sticky="ew", columnspan=2)

        self.button["DisconnectMQTT"] = tk.Button(self.root, text="DisconnectMQTT", padx=5,
                       command=self.DisconnectMQTT, state="disabled")
        # self.button["DisconnectMQTT"].grid(row=row,column=4,padx=2,pady=2,sticky="ew", columnspan=2)

        self.button["ToolChange"] = tk.Button(self.root, text="no task", padx=5,
                      command=None, state="disabled")
        self.button["ToolChange"].grid(
            row=row,column=6,padx=2,pady=2,sticky="ew", columnspan=2)

        self.frame_is_emergency_stopped = tk.Frame(self.root)
        self.frame_is_emergency_stopped.grid(row=row,column=8,padx=2,pady=2,sticky="w", columnspan=2)
        self.canvas_is_emergency_stopped = \
            tk.Canvas(self.frame_is_emergency_stopped, width=10, height=10)
        self.canvas_is_emergency_stopped.pack(side="left",padx=10)
        self.light_is_emergency_stopped = \
            self.canvas_is_emergency_stopped.create_oval(1, 1, 9, 9, fill="gray")
        self.label_is_emergency_stopped = \
            tk.Label(self.frame_is_emergency_stopped, text="EmergencyStopped")
        self.label_is_emergency_stopped.pack(side="left",padx=2)

        row += 1

        self.button["EnableRobot"] = tk.Button(self.root, text="EnableRobot", padx=5,
                      command=self.EnableRobot, state="disabled")
        self.button["EnableRobot"].grid(row=row,column=0,padx=2,pady=2,sticky="ew", columnspan=2)

        self.button["DisableRobot"] = tk.Button(self.root, text="DisableRobot", padx=5,
                      command=self.DisableRobot, state="disabled")
        self.button["DisableRobot"].grid(row=row,column=2,padx=2,pady=2,sticky="ew", columnspan=2)

        self.button["ReleaseHand"] = tk.Button(self.root, text="ReleaseHand", padx=5,
                      command=self.ReleaseHand, state="disabled")
        self.button["ReleaseHand"].grid(row=row,column=4,padx=2,pady=2,sticky="ew", columnspan=2)

        self.button["LineCut"] = tk.Button(self.root, text="no task ", padx=5,
                      command=None, state="disabled")
        self.button["LineCut"].grid(row=row,column=6,padx=2,pady=2,sticky="ew", columnspan=2)

        self.frame_enabled = tk.Frame(self.root)
        self.frame_enabled.grid(row=row,column=8,padx=2,pady=2,sticky="w", columnspan=2)
        self.canvas_enabled = \
            tk.Canvas(self.frame_enabled, width=10, height=10)
        self.canvas_enabled.pack(side="left",padx=10)
        self.light_enabled = \
            self.canvas_enabled.create_oval(1, 1, 9, 9, fill="gray")
        self.label_enabled = \
            tk.Label(self.frame_enabled, text="Enabled")
        self.label_enabled.pack(side="left",padx=2)

        row += 1

        self.button["SetAreaEnabled"] = tk.Button(self.root, text="SetAreaEnabled", padx=5,
                      command=self.SetArea, state="disabled")
        self.button["SetAreaEnabled"].grid(row=row,column=0,padx=2,pady=2,sticky="ew", columnspan=2)

        self.button["TidyPose"] = tk.Button(self.root, text="TidyPose", padx=5,
                      command=self.TidyPose, state="disabled")
        self.button["TidyPose"].grid(row=row,column=2,padx=2,pady=2,sticky="ew", columnspan=2)

        self.button["ClearError"] = tk.Button(self.root, text="ClearError", padx=5,
                      command=self.ClearError, state="disabled")
        self.button["ClearError"].grid(row=row,column=4,padx=2,pady=2,sticky="ew", columnspan=2)

        self.frame_error = tk.Frame(self.root)
        self.frame_error.grid(row=row,column=8,padx=2,pady=2,sticky="w", columnspan=2)
        self.canvas_error = \
            tk.Canvas(self.frame_error, width=10, height=10)
        self.canvas_error.pack(side="left",padx=10)
        self.light_error = \
            self.canvas_error.create_oval(1, 1, 9, 9, fill="gray")
        self.label_error = \
            tk.Label(self.frame_error, text="Error")
        self.label_error.pack(side="left",padx=2)

        row += 1

        self.button["StartMQTTControl"] = tk.Button(self.root, text="StartMQTTControl", padx=5,
                      command=self.StartMQTTControl, state="disabled")
        self.button["StartMQTTControl"].grid(
            row=row,column=0,padx=2,pady=2,sticky="ew", columnspan=2)
        
        self.button["StopMQTTControl"] = tk.Button(self.root, text="StopMQTTControl", padx=5,
                      command=self.StopMQTTControl, state="disabled")
        self.button["StopMQTTControl"].grid(
            row=row,column=2,padx=2,pady=2,sticky="ew", columnspan=2)

        self.button["ChangeLogFile"] = tk.Button(self.root, text="ChangeLogFile", padx=5,
                    command=self.ChangeLogFile, state="disabled")
        self.button["ChangeLogFile"].grid(
            row=row,column=4,padx=2,pady=2,sticky="ew", columnspan=2)        

        self.frame_mqtt_control = tk.Frame(self.root)
        self.frame_mqtt_control.grid(row=row,column=8,padx=2,pady=2,sticky="w", columnspan=2)
        self.canvas_mqtt_control = \
            tk.Canvas(self.frame_mqtt_control, width=10, height=10)
        self.canvas_mqtt_control.pack(side="left",padx=10)
        self.light_mqtt_control = \
            self.canvas_mqtt_control.create_oval(1, 1, 9, 9, fill="gray")
        self.label_mqtt_control = \
            tk.Label(self.frame_mqtt_control, text="MQTTControl")
        self.label_mqtt_control.pack(side="left",padx=2)

        # 加速機能つきボタン
        class AccelerateButton(tk.Button):
            def __init__(
                self,
                master,
                jog_callback,
                jog_args,
                accelerate_settings=[
                    {"elapsed": 0, "step": 0.1, "interval": 100},
                    {"elapsed": 1, "step": 1, "interval": 250},
                ],
                **kwargs,
            ):
                super().__init__(master, **kwargs)
                self.jog_callback = jog_callback
                self.jog_args = jog_args
                self.accelerate_settings = accelerate_settings
                self._after_id = None
                self._interval = 100  # ms 初期間隔
                self._step = 1  # 初期増分
                self._press_time = None
                self.bind('<ButtonPress-1>', self._on_press)
                self.bind('<ButtonRelease-1>', self._on_release)
                self.bind('<Leave>', self._on_release)

            def _on_press(self, event=None):
                self._press_time = datetime.datetime.now()
                self._repeat()

            def _on_release(self, event=None):
                if self._after_id:
                    self.after_cancel(self._after_id)
                    self._after_id = None
                self._press_time = None

            def _repeat(self):
                # 経過時間で加速
                if self._press_time:
                    elapsed = (datetime.datetime.now() - self._press_time).total_seconds()
                    for setting in self.accelerate_settings:
                        if elapsed >= setting["elapsed"]:
                            self._step = setting["step"]
                            self._interval = setting["interval"]
                    # コールバック呼び出し
                    self.jog_callback(*self.jog_args, step=self._step)
                    self._after_id = self.after(self._interval, self._repeat)

        # Joint Jog
        row += 1

        self.frame_area_enabled = tk.Frame(self.root)
        self.frame_area_enabled.grid(row=row,column=8,padx=2,pady=2,sticky="w", columnspan=2)
        self.canvas_area_enabled = \
            tk.Canvas(self.frame_area_enabled, width=10, height=10)
        self.canvas_area_enabled.pack(side="left",padx=10)
        self.light_area_enabled = \
            self.canvas_area_enabled.create_oval(1, 1, 9, 9, fill="gray")
        self.label_area_enabled = \
            tk.Label(self.frame_area_enabled, text="AreaEnabled")
        self.label_area_enabled.pack(side="left",padx=2)

        tk.Label(self.root, text="Joint Jog").grid(row=row, column=0, padx=2, pady=2, sticky="w")
        joint_names = ["J1", "J2", "J3", "J4", "J5", "J6"]
        joint_accelerate_settings = [
            {"elapsed": 0, "step": 0.1, "interval": 100},
            {"elapsed": 1, "step": 1, "interval": 250},
        ]
        self.button["joint_jog"] = {}
        for i, joint in enumerate(joint_names):
            frame = tk.Frame(self.root)
            frame.grid(row=row, column=2+i, padx=2, pady=2, sticky="ew")
            tk.Label(frame, text=joint, width=2, anchor="e").pack(side="left", padx=10)
            btn_minus = AccelerateButton(frame, self.jog_joint_accel, (i, -1), 
                                         accelerate_settings=joint_accelerate_settings, text="-", width=1, state="disabled")
            btn_minus.pack(side="left", expand=True, fill="x")
            btn_plus = AccelerateButton(frame, self.jog_joint_accel, (i, 1),
                                         accelerate_settings=joint_accelerate_settings, text="+", width=1, state="disabled")
            btn_plus.pack(side="left", expand=True, fill="x")
            self.button["joint_jog"][joint] = {"minus": btn_minus, "plus": btn_plus}
        # TCP Jog
        row += 1
        tk.Label(self.root, text="TCP Jog").grid(row=row, column=0, padx=2, pady=2, sticky="w")
        tcp_names = [" X", " Y", " Z", "RX", "RY", "RZ"]
        tcp_accelerate_settings = [
            {"elapsed": 0, "step": 0.1, "interval": 100},
            {"elapsed": 1, "step": 1, "interval": 250},
            {"elapsed": 2, "step": 10, "interval": 500},
        ]
        self.button["tcp_jog"] = {}
        for i, tcp in enumerate(tcp_names):
            frame = tk.Frame(self.root)
            frame.grid(row=row, column=2+i, padx=2, pady=2, sticky="ew")
            tk.Label(frame, text=tcp, width=2, anchor="e").pack(side="left", padx=10)
            btn_minus = AccelerateButton(frame, self.jog_tcp_accel, (i, -1),
                                         accelerate_settings=tcp_accelerate_settings, text="-", width=1, state="disabled")
            btn_minus.pack(side="left", expand=True, fill="x")
            btn_plus = AccelerateButton(frame, self.jog_tcp_accel, (i, 1),
                                         accelerate_settings=tcp_accelerate_settings, text="+", width=1, state="disabled")
            btn_plus.pack(side="left", expand=True, fill="x")
            self.button["tcp_jog"][tcp] = {"minus": btn_minus, "plus": btn_plus}
        row += 1

        tk.Label(self.root, text="State").grid(
            row=row, column=0, padx=2, pady=10, sticky="w", columnspan=4)
        self.string_var_states = {}
        for i in range(6):
            frame_state = tk.Frame(self.root)
            frame_state.grid(row=row+1+i, column=0, padx=2, pady=2, sticky="ew", columnspan=2)
            label_target = tk.Label(frame_state, text=f"J{i + 1}")
            label_target.pack(side="left", padx=10)
            string_var_state = tk.StringVar()
            string_var_state.set("")
            self.string_var_states[f"J{i + 1}"] = string_var_state
            text_box_state = tk.Label(
                frame_state,
                textvariable=string_var_state,
                bg="white",
                relief="solid",
                bd=1,
                anchor="e",
            )
            text_box_state.pack(side="right", padx=2, expand=True, fill="x")

        frame_state = tk.Frame(self.root)
        frame_state.grid(row=row+1, column=4, padx=2, pady=2, sticky="ew", columnspan=2)
        label_target = tk.Label(frame_state, text="Tool ID")
        label_target.pack(side="left", padx=10)
        string_var_state = tk.StringVar()
        string_var_state.set("")
        self.string_var_states["Tool ID"] = string_var_state
        text_box_state = tk.Label(
            frame_state,
            textvariable=string_var_state,
            bg="white",
            relief="solid",
            bd=1,
            anchor="e",
        )
        text_box_state.pack(side="right", padx=2, expand=True, fill="x")

        self.string_var_states_tcp = {}
        for i in range(6):
            frame_state = tk.Frame(self.root)
            frame_state.grid(row=row+1+i, column=2, padx=2, pady=2, sticky="ew", columnspan=2)
            label_target = tk.Label(frame_state, text=f"{tcp_names[i]}")
            label_target.pack(side="left", padx=10)
            string_var_state = tk.StringVar()
            string_var_state.set("")
            self.string_var_states_tcp[i] = string_var_state
            text_box_state = tk.Label(
                frame_state,
                textvariable=string_var_state,
                bg="white",
                relief="solid",
                bd=1,
                anchor="e",
            )
            text_box_state.pack(side="right", padx=2, expand=True, fill="x")

        tk.Label(self.root, text="Target").grid(
            row=row, column=6, padx=2, pady=2, sticky="w", columnspan=4)
        self.string_var_targets = {}
        for i in range(6):
            frame_target = tk.Frame(self.root)
            frame_target.grid(
                row=row+1+i, column=6, padx=2, pady=2, sticky="ew", columnspan=2)
            label_target = tk.Label(frame_target, text=f"J{i + 1}")
            label_target.pack(side="left", padx=10)
            string_var_target = tk.StringVar()
            string_var_target.set("")
            self.string_var_targets[f"J{i + 1}"] = string_var_target
            text_box_target = tk.Label(
                frame_target,
                textvariable=string_var_target,
                bg="white",
                relief="solid",
                bd=1,
                anchor="e",
            )
            text_box_target.pack(side="right", padx=2, expand=True, fill="x")

        frame_target = tk.Frame(self.root)
        frame_target.grid(row=row+1, column=8, padx=2, pady=2, sticky="ew", columnspan=2)
        label_target = tk.Label(frame_target, text="grip")
        label_target.pack(side="left", padx=10)
        string_var_target = tk.StringVar()
        string_var_target.set("")
        self.string_var_targets["grip"] = string_var_target
        text_box_target = tk.Label(
            frame_target,
            textvariable=string_var_target,
            bg="white",
            relief="solid",
            bd=1,
            anchor="e",
        )
        text_box_target.pack(side="right", padx=2, expand=True, fill="x")

        row += 7

#        tk.Label(self.root, text="Topics").grid(
#            row=row, column=0, padx=2, pady=10, sticky="w", columnspan=8)
        topic_types = [
            "mgr/register",
            "dev",
            "robot",
            "control",
        ]
        self.string_var_topics = {
            topic: tk.StringVar() for topic in topic_types}
        self.topic_monitors = {}
        for i, topic_type in enumerate(topic_types):
            frame_topic = tk.Frame(self.root)
            frame_topic.grid(
                row=row+1+3*i, column=0, padx=2, pady=2,
                sticky="ew", columnspan=10)
            label_topic_type = tk.Label(frame_topic, text=topic_type)
            label_topic_type.pack(side="left", padx=2)
            label_actual_topic = tk.Label(
                frame_topic, text="(Actual Topic)")
            label_actual_topic.pack(side="left", padx=2)
            string_var_topic = self.string_var_topics[topic_type]
            text_box_topic = tk.Label(
                frame_topic,
                textvariable=string_var_topic,
                bg="white",
                relief="solid",
                bd=1,
                anchor="w",
            )
            text_box_topic.pack(side="left", padx=2, expand=True, fill="x")
            frame_topic = tk.Frame(self.root)
            frame_topic.grid(
                row=row+2+3*i, column=0, padx=2, pady=2,
                sticky="ew", columnspan=10, rowspan=2)
            height = 3
            if topic_type in ["mgr/register", "dev"]:
                height = 2
            self.topic_monitors[topic_type] = scrolledtext.ScrolledText(
                frame_topic, height=height)
            self.topic_monitors[topic_type].pack(
                side="left", padx=2, expand=True, fill="both")

        row += 1 + 3*len(topic_types)

        frame_sm = tk.Frame(self.root)
        frame_sm.grid(
            row=row, column=0, padx=2, pady=10, sticky="ew", columnspan=10)
        label_sm = tk.Label(frame_sm, text="Shared Memory (rounded)")
        label_sm.pack(side="left", padx=2)
        self.string_var_sm = tk.StringVar()
        text_box_sm = tk.Label(
            frame_sm,
            textvariable=self.string_var_sm,
            bg="white",
            relief="solid",
            bd=1,
            anchor="w",
        )
        text_box_sm.pack(side="left", padx=2, expand=True, fill="x")

        row += 1

        tk.Label(self.root, text="Log Monitor").grid(
            row=row, column=0, padx=2, pady=2, sticky="w", columnspan=10)
        self.log_monitor = scrolledtext.ScrolledText(
            self.root, height=10)
        self.log_monitor.grid(
            row=row+1,column=0,padx=2,pady=2,columnspan=10, sticky="nsew")
        self.log_monitor.tag_config("INFO", foreground="black")
        self.log_monitor.tag_config("WARNING", foreground="orange")
        self.log_monitor.tag_config("ERROR", foreground="red")
        self.update_monitor()
        self.start_update_gui_log()
        self.update_button_states_from_mqtt_control()

    def get_logging_dir(self):
        now = datetime.datetime.now()
        log_dir = "log"
        os.makedirs(log_dir, exist_ok=True)
        date_str = now.strftime("%Y-%m-%d")
        os.makedirs(os.path.join(log_dir, date_str), exist_ok=True)
        time_str = now.strftime("%H-%M-%S")
        logging_dir = os.path.join(log_dir, date_str, time_str)
        os.makedirs(logging_dir, exist_ok=True)
        return logging_dir

    def setup_logging(
        self,
        log_queue: Optional[multiprocessing.Queue] = None,
        logging_dir: Optional[str] = None,
    ) -> None:
        """複数プロセスからのログを集約する方法を設定する"""
        if logging_dir is None:
            logging_dir = self.get_logging_dir()
        handlers = [
            logging.FileHandler(os.path.join(logging_dir, "log.txt")),
            logging.StreamHandler(),
        ]
        if log_queue is not None:
            handlers.append(GUILoggingHandler(self))
        formatter = MicrosecondFormatter(
            "[%(asctime)s][%(name)s][%(levelname)s] %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S.%f",
        )
        for handler in handlers:
            handler.setFormatter(formatter)
        self.listener = logging.handlers.QueueListener(log_queue, *handlers)
        # listener.startからroot.mainloopまでのわずかな間は、
        # GUIの更新がないが、root.mainloopが始まると溜まっていたログも表示される
        self.listener.start()

    def setup_logger(
        self, log_queue: Optional[multiprocessing.Queue] = None,
    ) -> None:
        """GUIプロセスからのログの設定"""
        self.logger = logging.getLogger("GUI")
        if log_queue is not None:
            self.handler = logging.handlers.QueueHandler(log_queue)
        else:
            self.handler = logging.StreamHandler()
        self.logger.addHandler(self.handler)
        self.logger.setLevel(logging.INFO)

    def ConnectRobot(self):
        

        if self.pm.state_monitor:
        # if self.pm.state_control and self.pm.state_monitor:
            return
        # self.pm.startControl(logging_dir=self.logging_dir)
        self.pm.startMonitor(logging_dir=self.logging_dir)
        if self.use_joint_monitor_plot:
            self.pm.startMonitorGUI()
        self.button["ConnectRobot"].config(state="disabled")
        self.button["ClearError"].config(state="normal")
        self.button["SetAreaEnabled"].config(state="normal")
        self.button["DisableRobot"].config(state="normal")
        self.button["EnableRobot"].config(state="normal")
        self.button["ReleaseHand"].config(state="normal")
        self.button["TidyPose"].config(state="normal")
        self.button["ToolChange"].config(state="normal")
        self.button["ChangeLogFile"].config(state="normal")
        self.button["DemoPutDownBox"].config(state="normal")
        self.button["LineCut"].config(state="normal")
        for joint in self.button["joint_jog"]:
            self.button["joint_jog"][joint]["minus"].config(state="normal")
            self.button["joint_jog"][joint]["plus"].config(state="normal")
        for tcp in self.button["tcp_jog"]:
            self.button["tcp_jog"][tcp]["minus"].config(state="normal")
            self.button["tcp_jog"][tcp]["plus"].config(state="normal")
        if self.pm.state_recv_mqtt:
            self.button["StartMQTTControl"].config(state="normal")
            self.button["StopMQTTControl"].config(state="normal")

    def ConnectMQTT(self):
        if self.pm.state_recv_mqtt:
            return
        self.pm.startRecvMQTT()
        self.button["ConnectMQTT"].config(state="disabled")
        self.button["DisconnectMQTT"].config(state="normal")
        self.button["ClearError"].config(state="normal")
        self.button["SetAreaEnabled"].config(state="normal")
        self.button["DisableRobot"].config(state="normal")
        self.button["EnableRobot"].config(state="normal")
        self.button["ReleaseHand"].config(state="normal")
        self.button["TidyPose"].config(state="normal")
        if self.pm.state_control and self.pm.state_monitor:
            self.button["StartMQTTControl"].config(state="normal")
            self.button["StopMQTTControl"].config(state="normal")

    def temporarily_disable_button(self):
        self.disable_keys = []
        # ボタンを一時的に無効化
        for key in self.button:
            if isinstance(self.button[key], dict):
                for axis in self.button[key]:
                    for polarity in self.button[key][axis]:
                        state = self.button[key][axis][polarity].cget("state")
                        if state == "normal":
                            self.disable_keys.append((key, axis, polarity))
                            self.button[key][axis][polarity].config(
                                state="disabled")
            else:
                state = self.button[key].cget("state")
                if state == "normal":
                    self.disable_keys.append(key)
                    self.button[key].config(state="disabled")
        self.root.update_idletasks()

    def recover_temporarily_disable_button(self):
        # 元の状態に復元
        for key in self.disable_keys:
            if isinstance(key, tuple):
                self.button[key[0]][key[1]][key[2]].config(state="normal")
            else:
                self.button[key].config(state="normal")

    def _exclusive_button_action(self, action):
        # ボタンを一時的に無効化して、actionを別スレッドで実行し、
        # action終了後にボタンを元の状態に戻す
        # action終了が重要な場合はactionは同期処理である必要がある
        # またactionすなわちサブスレッドで
        # プロセスを作成することは予期せぬ問題の原因になることがあるとのことなので非推奨
        self.temporarily_disable_button()
        def _target():
            action()
            # GUI処理はメインスレッドに呼び出す
            self.root.after(0, self.recover_temporarily_disable_button)
        threading.Thread(target=_target).start()

    def EnableRobot(self):
        if not self.pm.state_control:
            return
        self._exclusive_button_action(self.pm.enable)

    def DisableRobot(self):
        if not self.pm.state_control:
            return
        self._exclusive_button_action(self.pm.disable)

    def SetArea(self):
        if not self.pm.state_control:
            return
        popup = SetAreaPopup(self.root)
        enabled = popup.result
        if enabled is None:
            return
        self._exclusive_button_action(lambda: self.pm.set_area_enabled(enabled))

    def TidyPose(self):
        if not self.pm.state_control:
            return
        self._exclusive_button_action(self.pm.tidy_pose)

    def ClearError(self):
        if not self.pm.state_control:
            return
        self._exclusive_button_action(self.pm.clear_error)

    def StartMQTTControl(self):
        if ((not self.pm.state_control) or
            (not self.pm.state_monitor) or
            (not self.pm.state_recv_mqtt)):
            return
        self._exclusive_button_action(self.pm.start_mqtt_control)

    def StopMQTTControl(self):
        if ((not self.pm.state_control) or
            (not self.pm.state_monitor) or
            (not self.pm.state_recv_mqtt)):
            return
        self._exclusive_button_action( self.pm.stop_mqtt_control)

    def ReleaseHand(self):
        if not self.pm.state_control:
            return
        self._exclusive_button_action(self.pm.release_hand)
    
    def LineCut(self):
        if not self.pm.state_control:
            return
        self._exclusive_button_action(self.pm.line_cut)

    def ToolChange(self):
        if not self.pm.state_control:
            return
        popup = ToolChangePopup(self.root)
        tool_id = popup.result
        if tool_id is None:
            return
        self._exclusive_button_action(lambda: self.pm.tool_change(tool_id))

    def jog_joint(self, joint, direction):
        if not self.pm.state_control:
            return
        self.pm.jog_joint(joint, direction)

    def jog_tcp(self, axis, direction):
        if not self.pm.state_control:
            return
        self.pm.jog_tcp(axis, direction)

    def jog_joint_accel(self, joint, direction, step=1):
        # 加速対応のジョイントジョグコールバック
        self.jog_joint(joint, direction * step)

    def jog_tcp_accel(self, axis, direction, step=1):
        # 加速対応のTCPジョグコールバック
        self.jog_tcp(axis, direction * step)

    def DemoPutDownBox(self):
        if not self.pm.state_control:
            return
        self._exclusive_button_action(self.pm.demo_put_down_box)

    def DisconnectMQTT(self):
        print("Disconnect MQTT!!")

    def ChangeLogFile(self):
        if getattr(self, "listener", None) is not None:
            self.listener.stop()
        logging_dir = self.get_logging_dir()
        self.pm.change_log_file(logging_dir)
        # ここでのロガーメッセージが古いファイルか新しいファイルに記録されるかは
        # タイミングによって異なることがある
        self.logger.info(f"Change log directory to {logging_dir}")
        self.logger.info("Change log file")
        # サブプロセスの制御値、状態値のファイルの保存先の変更完了を待つ
        while True:
            if self.pm.ar[33] == 0 and self.pm.ar[34] == 0:
                break
            time.sleep(0.1)
        self.setup_logging(
            log_queue=self.pm.log_queue, logging_dir=logging_dir)

    def update_gui_log(self):
        while True:
            msgs = []
            try:
                for _ in range(10):
                    msg = self.gui_log_queue.get(block=False)
                    msgs.append(msg)
            except queue.Empty:
                pass
            if not msgs:
                msg = self.gui_log_queue.get(block=True, timeout=None)
                msgs.append(msg)
            for msg in msgs:
                self.root.after(0, self.update_log, msg)

    def start_update_gui_log(self):
        self.update_gui_log_thread = threading.Thread(
            target=self.update_gui_log, daemon=True)
        self.update_gui_log_thread.start()

    def update_button_states_from_mqtt_control(self):
        # MQTT制御に入れる状態にならなければチェックしない
        if not self.pm.state_control or not self.pm.state_monitor:
            self.root.after(1000, self.update_button_states_from_mqtt_control)
            return
        # MQTT制御に入れる状態であれば、MQTT制御状態に応じてボタンの有効無効を切り替える
        last_state_mqtt_control = getattr(
            self, "last_state_mqtt_control", False)
        state_mqtt_control = self.pm.state_mqtt_control
        # 前回の状態と異なる場合のみ切り替えて負荷を下げる
        if state_mqtt_control != last_state_mqtt_control:
            kind1 = "disabled" if state_mqtt_control else "normal"
            kind2 = "normal" if state_mqtt_control else "disabled"
            self.button["StartMQTTControl"].config(state=kind1)
            self.button["StopMQTTControl"].config(state=kind2)
            self.button["ClearError"].config(state=kind1)
            self.button["SetAreaEnabled"].config(state=kind1)
            self.button["DisableRobot"].config(state=kind1)
            self.button["EnableRobot"].config(state=kind1)
            self.button["ReleaseHand"].config(state=kind1)
            self.button["TidyPose"].config(state=kind1)
            self.button["ToolChange"].config(state=kind1)
            self.button["ChangeLogFile"].config(state=kind1)
            self.button["DemoPutDownBox"].config(state=kind1)
            self.button["LineCut"].config(state=kind1)
            for joint in self.button["joint_jog"]:
                self.button["joint_jog"][joint]["minus"].config(state=kind1)
                self.button["joint_jog"][joint]["plus"].config(state=kind1)
            for tcp in self.button["tcp_jog"]:
                self.button["tcp_jog"][tcp]["minus"].config(state=kind1)
                self.button["tcp_jog"][tcp]["plus"].config(state=kind1)
            self.last_state_mqtt_control = state_mqtt_control
        self.root.after(1000, self.update_button_states_from_mqtt_control)

    def update_monitor(self):
        # モニタープロセスからの情報
        start_time = time.time()

        log = self.pm.get_current_monitor_log()
        if log:
            # ロボットの姿勢情報が流れるトピック
            topic_type = log.pop("topic_type")
            topic = log.pop("topic")
            poses = log.pop("poses", None)
            log_str = json.dumps(log, ensure_ascii=False)
            self.string_var_topics[topic_type].set(topic)
            self.update_topic(log_str, self.topic_monitors[topic_type])

            # 各情報をパース
            color = "red" if log.get("emergency_stopped") else "gray"
            self.canvas_is_emergency_stopped.itemconfig(self.light_is_emergency_stopped, fill=color)
            color = "lime" if log.get("area_enabled") else "gray"
            self.canvas_area_enabled.itemconfig(self.light_area_enabled, fill=color)
            color = "lime" if log.get("enabled") else "gray"
            self.canvas_enabled.itemconfig(self.light_enabled, fill=color)
            color = "lime" if log.get("mqtt_control") == "ON" else "gray"
            self.canvas_mqtt_control.itemconfig(
                self.light_mqtt_control, fill=color)
            color = "red" if "error" in log else "gray"
            self.canvas_error.itemconfig(self.light_error, fill=color)
            joints = log.get("joints")
            if joints is not None:
                for i in range(6):
                    self.string_var_states[f"J{i + 1}"].set(f"{joints[i]:.2f}")
            else:
                for i in range(6):
                    self.string_var_states[f"J{i + 1}"].set("")
            tool_id = log.get("tool_id")
            if tool_id is not None:
                self.string_var_states["Tool ID"].set(f"{tool_id}")
            else:
                self.string_var_states["Tool ID"].set("")
            if poses is not None:
                for i in range(6):
                    self.string_var_states_tcp[i].set(f"{poses[i]:.2f}")

        # MQTT制御プロセスからの情報
        log = self.pm.get_current_mqtt_control_log()
        if log:
            topic_type = log.pop("topic_type")
            topic = log.pop("topic")
            log_str = json.dumps(log, ensure_ascii=False)
            self.string_var_topics[topic_type].set(topic)
            self.update_topic(log_str, self.topic_monitors[topic_type])

            joints = log.get("joints")
            if joints is not None:
                for i in range(6):
                    self.string_var_targets[f"J{i + 1}"].set(f"{joints[i]:.2f}")
            else:
                for i in range(6):
                    self.string_var_targets[f"J{i + 1}"].set("")
            grip = log.get("grip")
            if grip is not None:
                self.string_var_targets["grip"].set(f"{grip}")
            else:
                self.string_var_targets["grip"].set("")
        
        # 共有メモリの情報
        sm = self.pm.ar.copy()
        sm_str = ",".join(str(int(round(x))) for x in sm)
        self.string_var_sm.set(sm_str)
        elapsed = time.time() - start_time
        if elapsed > 0.1:
            self.logger.warning(f"Monitor process took {elapsed:.3f}s")
        self.root.after(300, self.update_monitor)  # 100ms間隔で表示を更新

    def update_topic(self, msg: str, box: scrolledtext.ScrolledText) -> None:
        """トピックの内容を表示する"""
        box.delete("1.0", tk.END)
        box.insert(tk.END, msg + "\n")  # ログを表示

    def update_log(self, msg: str) -> None:
        """ログを表示する"""
        box = self.log_monitor

        ## 最後の行を表示しているときだけ自動スクロールする
        # 挿入前にスクロールバーが一番下かどうかを判定
        # yview()は(最初に表示されている行, 最後に表示されている行)を0.0～1.0で返す
        at_bottom = False
        if box.yview()[1] >= 0.999:  # 浮動小数点の誤差を考慮
            at_bottom = True
        # ログレベルごとに色を変える
        if "[ERROR]" in msg:
            tag = "ERROR"
        elif "[WARNING]" in msg:
            tag = "WARNING"
        else:
            tag = "INFO"
        box.insert(tk.END, msg + "\n", tag)  # ログを表示
        # 挿入後、元々一番下にいた場合のみ自動スクロール
        if at_bottom:
            box.see(tk.END)
        
        # 行数が多すぎる場合は古い行を一部削除してメモリを節約
        current_lines = int(box.index('end-1c').split('.')[0])
        if current_lines > 1000:
            excess_lines = current_lines - 1000
            box.delete("1.0", f"{excess_lines}.0")

    def on_closing(self):
        """ウインドウを閉じるときの処理"""
        self.pm.stop_all_processes()
        self.listener.stop()
        self.handler.close()
        logging.shutdown()
        self.root.destroy()


if __name__ == '__main__':
    # Freeze Support for Windows
    multiprocessing.freeze_support()


    # NOTE: 現在ロボットに付いているツールが何かを管理する方法がないので
    # ロボット制御コードの使用者に指定してもらう
    # ツールによっては、ツールとの通信が不要なものがあるので、通信の成否では判定できない
    # 現在のツールの状態を常にファイルに保存しておき、ロボット制御コードを再起動するときに
    # そのファイルを読み込むようにすれば管理はできるが、エラーで終了したときに
    # ファイルの情報が正確かいまのところ保証できないので、指定してもらう
    import argparse
    parser = argparse.ArgumentParser()
#    parser.add_argument(
#        "--tool-id",
#        type=int,
#        required=True,
#        choices=tool_ids,
#        help="現在ロボットに付いているツールのID",
#   )
    parser.add_argument(
        "--use-joint-monitor-plot",
        action="store_true",
        help="関節角度のモニタープロットを使用する",
    )
    args = parser.parse_args()
    kwargs = vars(args)
    import os
    # HACK: コードの変化を少なくするため、
    # ロボット制御プロセスに引数で渡すのではなく環境変数で渡す

#    os.environ["TOOL_ID"] = str(kwargs.pop("tool_id"))

    root = tk.Tk()
    mqwin = MQTTWin(root, **kwargs)
    mqwin.root.lift()
    root.protocol("WM_DELETE_WINDOW", mqwin.on_closing)
    root.mainloop()
