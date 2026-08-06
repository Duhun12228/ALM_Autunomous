#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ALM Scan Context / ICP Experiment GUI

- Jetson + ROS 2 Humble 환경에서는 ROS 명령을 직접 실행합니다.
- ROS가 없는 PC에서는 자동으로 "미리보기 모드"가 켜져 GUI와 생성 명령만 확인할 수 있습니다.
- 표준 라이브러리 tkinter 기반이라 GUI 자체는 별도 pip 패키지 없이 실행 가능합니다.
- YAML 메시지 파싱을 사용하는 자동 Coarse-to-Fine 기능은 Jetson에서 python3-yaml이 필요합니다.

권장 실행:
    python3 alm_experiment_gui.py

Jetson 의존성:
    sudo apt install python3-tk python3-yaml
"""

from __future__ import annotations

import csv
import json
import math
import os
import platform
import queue
import shlex
import shutil
import signal
import subprocess
import threading
import time
import uuid
from dataclasses import asdict, dataclass
from datetime import datetime
from pathlib import Path
from typing import Callable, Dict, List, Optional, Sequence, Tuple

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    import yaml  # type: ignore
except Exception:
    yaml = None


APP_TITLE = "ALM Scan Context–ICP 실험 GUI"
DEFAULT_WS = "~/ALM_Autunomous/ALM_auto_ws"
DEFAULT_MAP = "~/ALM_Autunomous/ALM_auto_ws/src/alm_navigation/maps/alm_3d_map.pcd"
DEFAULT_DB = "~/ALM_Autunomous/ALM_auto_ws/src/alm_navigation/maps/sc_db.npz"
DEFAULT_FASTLIO_CONFIG = (
    "~/ALM_Autunomous/ALM_auto_ws/src/alm_navigation/config/"
    "fastlio_relocalization.yaml"
)


def expand_path(value: str) -> str:
    return str(Path(value).expanduser().resolve())


def shell_quote(value: str) -> str:
    return shlex.quote(str(value))


def now_text() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def run_id() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:6]


def bool_text(value: bool) -> str:
    return "성공" if value else "실패"


@dataclass
class ExperimentResult:
    timestamp: str
    experiment: str
    setting: str
    repeat: int
    sc_rank: str = ""
    selected_rank: str = ""
    sc_distance: str = ""
    icp_score: str = ""
    icp_rmse_m: str = ""
    elapsed_sec: str = ""
    auto_status: str = ""
    manual_judgement: str = ""
    note: str = ""


class ProcessManager:
    """Linux/Jetson에서 ROS 프로세스를 process group 단위로 관리합니다."""

    def __init__(self, log_callback: Callable[[str], None]) -> None:
        self.log_callback = log_callback
        self.processes: Dict[str, subprocess.Popen[str]] = {}
        self.lock = threading.Lock()

    def start(
        self,
        name: str,
        shell_command: str,
        cwd: Optional[str] = None,
        on_exit: Optional[Callable[[int], None]] = None,
    ) -> subprocess.Popen[str]:
        with self.lock:
            old = self.processes.get(name)
            if old and old.poll() is None:
                raise RuntimeError(f"이미 실행 중인 프로세스입니다: {name}")

        kwargs = {
            "stdout": subprocess.PIPE,
            "stderr": subprocess.STDOUT,
            "text": True,
            "bufsize": 1,
            "cwd": cwd,
        }
        if os.name == "posix":
            kwargs["preexec_fn"] = os.setsid

        proc = subprocess.Popen(
            ["bash", "-lc", shell_command],
            **kwargs,  # type: ignore[arg-type]
        )

        with self.lock:
            self.processes[name] = proc

        self.log_callback(f"[프로세스 시작] {name} (PID {proc.pid})")

        def reader() -> None:
            try:
                assert proc.stdout is not None
                for line in proc.stdout:
                    self.log_callback(f"[{name}] {line.rstrip()}")
            finally:
                code = proc.wait()
                self.log_callback(f"[프로세스 종료] {name} (code={code})")
                with self.lock:
                    if self.processes.get(name) is proc:
                        self.processes.pop(name, None)
                if on_exit:
                    on_exit(code)

        threading.Thread(target=reader, daemon=True).start()
        return proc

    def stop(self, name: str, timeout: float = 3.0) -> None:
        with self.lock:
            proc = self.processes.get(name)
        if not proc or proc.poll() is not None:
            return

        self.log_callback(f"[프로세스 중지] {name}")
        try:
            if os.name == "posix":
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            else:
                proc.terminate()
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            try:
                if os.name == "posix":
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                else:
                    proc.terminate()
                proc.wait(timeout=2.0)
            except Exception:
                try:
                    if os.name == "posix":
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    else:
                        proc.kill()
                except Exception:
                    pass
        finally:
            with self.lock:
                self.processes.pop(name, None)

    def stop_all(self) -> None:
        with self.lock:
            names = list(self.processes.keys())
        for name in reversed(names):
            self.stop(name)

    def is_running(self, name: str) -> bool:
        with self.lock:
            proc = self.processes.get(name)
        return bool(proc and proc.poll() is None)


class ResultTable(ttk.Frame):
    COLUMNS = (
        "timestamp",
        "setting",
        "repeat",
        "sc_rank",
        "selected_rank",
        "sc_distance",
        "icp_score",
        "rmse",
        "elapsed",
        "status",
        "judgement",
        "note",
    )

    HEADINGS = {
        "timestamp": "시간",
        "setting": "설정",
        "repeat": "회차",
        "sc_rank": "SC 순위",
        "selected_rank": "선택 순위",
        "sc_distance": "SC 거리",
        "icp_score": "ICP 점수",
        "rmse": "RMSE(m)",
        "elapsed": "시간(s)",
        "status": "자동 상태",
        "judgement": "수동 판정",
        "note": "메모",
    }

    def __init__(self, master: tk.Misc, experiment_name: str) -> None:
        super().__init__(master)
        self.experiment_name = experiment_name
        self.rows: List[ExperimentResult] = []

        self.tree = ttk.Treeview(
            self,
            columns=self.COLUMNS,
            show="headings",
            height=9,
        )
        for col in self.COLUMNS:
            self.tree.heading(col, text=self.HEADINGS[col])
            width = 85
            if col in ("timestamp", "setting", "note"):
                width = 130
            self.tree.column(col, width=width, anchor="center", stretch=True)

        yscroll = ttk.Scrollbar(self, orient="vertical", command=self.tree.yview)
        xscroll = ttk.Scrollbar(self, orient="horizontal", command=self.tree.xview)
        self.tree.configure(yscrollcommand=yscroll.set, xscrollcommand=xscroll.set)

        self.tree.grid(row=0, column=0, sticky="nsew")
        yscroll.grid(row=0, column=1, sticky="ns")
        xscroll.grid(row=1, column=0, sticky="ew")

        button_frame = ttk.Frame(self)
        button_frame.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(6, 0))
        for text, judgement in (
            ("정확함", "정확함"),
            ("약간 벗어남", "약간 벗어남"),
            ("오정합", "오정합"),
            ("실패", "실패"),
        ):
            ttk.Button(
                button_frame,
                text=text,
                command=lambda j=judgement: self.set_judgement(j),
            ).pack(side="left", padx=3)

        ttk.Button(
            button_frame, text="선택 행 삭제", command=self.delete_selected
        ).pack(side="right", padx=3)

        self.columnconfigure(0, weight=1)
        self.rowconfigure(0, weight=1)

    def add(self, result: ExperimentResult) -> None:
        self.rows.append(result)
        self.tree.insert(
            "",
            "end",
            iid=str(len(self.rows) - 1),
            values=(
                result.timestamp,
                result.setting,
                result.repeat,
                result.sc_rank,
                result.selected_rank,
                result.sc_distance,
                result.icp_score,
                result.icp_rmse_m,
                result.elapsed_sec,
                result.auto_status,
                result.manual_judgement,
                result.note,
            ),
        )

    def set_judgement(self, judgement: str) -> None:
        selection = self.tree.selection()
        if not selection:
            messagebox.showinfo("선택 필요", "판정할 결과 행을 선택하세요.")
            return
        for iid in selection:
            idx = int(iid)
            if idx >= len(self.rows):
                continue
            self.rows[idx].manual_judgement = judgement
            values = list(self.tree.item(iid, "values"))
            values[10] = judgement
            self.tree.item(iid, values=values)

    def delete_selected(self) -> None:
        for iid in self.tree.selection():
            self.tree.delete(iid)

    def export_rows(self) -> List[ExperimentResult]:
        visible: List[ExperimentResult] = []
        for iid in self.tree.get_children():
            idx = int(iid)
            if 0 <= idx < len(self.rows):
                visible.append(self.rows[idx])
        return visible


class ALMExperimentGUI(tk.Tk):
    def __init__(self) -> None:
        super().__init__()
        self.title(APP_TITLE)
        self.geometry("1460x920")
        self.minsize(1180, 760)

        self.log_queue: "queue.Queue[str]" = queue.Queue()
        self.results: Dict[str, ResultTable] = {}
        self.process_manager = ProcessManager(self.enqueue_log)
        self.last_command = ""
        self.experiment_start_time: Dict[str, float] = {}
        self.current_repeat: Dict[str, int] = {}

        self._configure_style()
        self._build_variables()
        self._build_layout()
        self.after(100, self._drain_log_queue)
        self.protocol("WM_DELETE_WINDOW", self.on_close)

        self.detect_environment()

    # ------------------------------------------------------------------
    # UI setup
    # ------------------------------------------------------------------
    def _configure_style(self) -> None:
        style = ttk.Style(self)
        available = style.theme_names()
        if "clam" in available:
            style.theme_use("clam")
        style.configure("Header.TLabel", font=("TkDefaultFont", 15, "bold"))
        style.configure("SubHeader.TLabel", font=("TkDefaultFont", 11, "bold"))
        style.configure("Status.TLabel", padding=6)

    def _build_variables(self) -> None:
        self.ws_var = tk.StringVar(value=DEFAULT_WS)
        self.map_var = tk.StringVar(value=DEFAULT_MAP)
        self.db_var = tk.StringVar(value=DEFAULT_DB)
        self.fastlio_config_var = tk.StringVar(value=DEFAULT_FASTLIO_CONFIG)
        self.output_var = tk.StringVar(
            value="~/ALM_Autunomous/experiment_runs"
        )
        self.branch_var = tk.StringVar(value="dev/fastlio2-sc")
        self.ros_setup_var = tk.StringVar(value="/opt/ros/humble/setup.bash")

        ros_available = shutil.which("ros2") is not None
        self.preview_var = tk.BooleanVar(value=not ros_available)
        self.auto_start_sensor_var = tk.BooleanVar(value=False)
        self.status_var = tk.StringVar(value="환경 확인 전")
        self.sensor_status_var = tk.StringVar(value="센서 미실행")

        self.common_accum_frames = tk.IntVar(value=10)
        self.common_topk = tk.IntVar(value=25)
        self.common_candidates = tk.IntVar(value=5)
        self.common_timeout = tk.DoubleVar(value=12.0)

        # Baseline(I1/T1) 기본값은 alm_navigation/launch/localization.launch.py 의
        # icp_node 파라미터와 동일하게 맞춘다. 이 값이 현재 브랜치의 실제 기준선이다.
        self.icp_map_voxel = tk.DoubleVar(value=0.5)
        self.icp_cloud_voxel = tk.DoubleVar(value=0.3)
        self.icp_iterations = tk.IntVar(value=75)
        self.icp_corr = tk.DoubleVar(value=1.0)
        self.icp_fitness = tk.DoubleVar(value=0.7)
        self.icp_count = tk.IntVar(value=10)
        self.icp_ransac = tk.DoubleVar(value=1.0)

        self.coarse_map_voxel = tk.DoubleVar(value=0.5)
        self.coarse_cloud_voxel = tk.DoubleVar(value=0.5)
        self.coarse_iterations = tk.IntVar(value=40)
        self.coarse_corr = tk.DoubleVar(value=1.0)
        self.coarse_fitness = tk.DoubleVar(value=0.5)

        self.fine_map_voxel = tk.DoubleVar(value=0.25)
        self.fine_cloud_voxel = tk.DoubleVar(value=0.25)
        self.fine_iterations = tk.IntVar(value=40)
        self.fine_corr = tk.DoubleVar(value=0.3)
        self.fine_fitness = tk.DoubleVar(value=0.2)

        self.initial_x_var = tk.DoubleVar(value=0.0)
        self.initial_y_var = tk.DoubleVar(value=0.0)
        self.initial_z_var = tk.DoubleVar(value=0.0)
        self.initial_yaw_var = tk.DoubleVar(value=0.0)

        self.db_num_ring = tk.IntVar(value=20)
        self.db_num_sector = tk.IntVar(value=60)
        self.db_radius = tk.DoubleVar(value=10.0)
        self.db_keyframe_z = tk.DoubleVar(value=0.0)
        self.db_selftest = tk.IntVar(value=20)

    def _build_layout(self) -> None:
        header = ttk.Frame(self, padding=(12, 8))
        header.pack(fill="x")
        ttk.Label(header, text=APP_TITLE, style="Header.TLabel").pack(side="left")
        ttk.Checkbutton(
            header,
            text="미리보기 모드(명령만 생성)",
            variable=self.preview_var,
        ).pack(side="right", padx=8)
        ttk.Label(
            header,
            textvariable=self.status_var,
            style="Status.TLabel",
        ).pack(side="right", padx=8)

        self.notebook = ttk.Notebook(self)
        self.notebook.pack(fill="both", expand=True, padx=10, pady=(0, 8))

        self._build_common_tab()
        self._build_baseline_tab()
        self._build_height_tab()
        self._build_step_tab()
        self._build_icp_tab()
        self._build_accum_tab()
        self._build_timeout_tab()
        self._build_final_tab()

        log_frame = ttk.LabelFrame(self, text="통합 실행 로그", padding=6)
        log_frame.pack(fill="both", expand=False, padx=10, pady=(0, 10))
        self.log_text = tk.Text(
            log_frame,
            height=11,
            wrap="none",
            font=("TkFixedFont", 9),
            state="disabled",
        )
        yscroll = ttk.Scrollbar(log_frame, orient="vertical", command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=yscroll.set)
        self.log_text.pack(side="left", fill="both", expand=True)
        yscroll.pack(side="right", fill="y")

    def _new_tab(self, title: str) -> ttk.Frame:
        frame = ttk.Frame(self.notebook, padding=10)
        self.notebook.add(frame, text=title)
        return frame

    def _path_row(
        self,
        parent: tk.Misc,
        row: int,
        label: str,
        variable: tk.StringVar,
        choose_file: bool = False,
        choose_dir: bool = False,
    ) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=4)
        ttk.Entry(parent, textvariable=variable, width=90).grid(
            row=row, column=1, sticky="ew", padx=6, pady=4
        )

        def browse() -> None:
            current = str(Path(variable.get()).expanduser())
            if choose_dir:
                selected = filedialog.askdirectory(initialdir=current)
            elif choose_file:
                selected = filedialog.askopenfilename(initialdir=str(Path(current).parent))
            else:
                selected = ""
            if selected:
                variable.set(selected)

        if choose_file or choose_dir:
            ttk.Button(parent, text="찾기", command=browse).grid(
                row=row, column=2, padx=4, pady=4
            )

    def _spin_row(
        self,
        parent: tk.Misc,
        row: int,
        label: str,
        variable: tk.Variable,
        from_: float,
        to: float,
        increment: float,
        width: int = 12,
    ) -> None:
        ttk.Label(parent, text=label).grid(row=row, column=0, sticky="w", pady=3)
        ttk.Spinbox(
            parent,
            textvariable=variable,
            from_=from_,
            to=to,
            increment=increment,
            width=width,
        ).grid(row=row, column=1, sticky="w", padx=6, pady=3)

    def _build_common_tab(self) -> None:
        tab = self._new_tab("공통 설정")
        tab.columnconfigure(0, weight=1)

        paths = ttk.LabelFrame(tab, text="Jetson / ROS 환경", padding=10)
        paths.grid(row=0, column=0, sticky="ew")
        paths.columnconfigure(1, weight=1)
        self._path_row(paths, 0, "워크스페이스", self.ws_var, choose_dir=True)
        self._path_row(paths, 1, "Prior map PCD", self.map_var, choose_file=True)
        self._path_row(paths, 2, "Scan Context DB", self.db_var, choose_file=True)
        self._path_row(
            paths, 3, "FAST-LIO 설정 YAML", self.fastlio_config_var, choose_file=True
        )
        self._path_row(paths, 4, "결과 저장 폴더", self.output_var, choose_dir=True)
        self._path_row(paths, 5, "ROS setup", self.ros_setup_var, choose_file=True)

        ttk.Label(paths, text="브랜치 기록").grid(row=6, column=0, sticky="w", pady=4)
        ttk.Combobox(
            paths,
            textvariable=self.branch_var,
            values=("dev/fastlio2-sc", "dev/sc-lio-sam"),
            state="readonly",
            width=25,
        ).grid(row=6, column=1, sticky="w", padx=6, pady=4)

        runtime = ttk.LabelFrame(tab, text="공통 SC 실행값", padding=10)
        runtime.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        self._spin_row(runtime, 0, "누적 프레임", self.common_accum_frames, 1, 100, 1)
        self._spin_row(runtime, 1, "topk", self.common_topk, 1, 500, 1)
        self._spin_row(runtime, 2, "ICP 후보 수", self.common_candidates, 1, 100, 1)
        self._spin_row(runtime, 3, "후보 timeout(s)", self.common_timeout, 0.5, 120, 0.5)

        buttons = ttk.Frame(tab)
        buttons.grid(row=2, column=0, sticky="ew", pady=10)
        ttk.Button(
            buttons, text="환경 검사", command=self.detect_environment
        ).pack(side="left", padx=4)
        ttk.Button(
            buttons, text="센서 시작", command=self.start_sensor
        ).pack(side="left", padx=4)
        ttk.Button(
            buttons, text="RViz 시작", command=self.start_rviz
        ).pack(side="left", padx=4)
        ttk.Button(
            buttons, text="전체 프로세스 중지", command=self.stop_all
        ).pack(side="left", padx=4)
        ttk.Button(
            buttons, text="설정 저장", command=self.save_gui_config
        ).pack(side="right", padx=4)
        ttk.Button(
            buttons, text="설정 불러오기", command=self.load_gui_config
        ).pack(side="right", padx=4)

        status = ttk.LabelFrame(tab, text="상태", padding=10)
        status.grid(row=3, column=0, sticky="ew")
        ttk.Label(status, text="센서:").grid(row=0, column=0, sticky="w")
        ttk.Label(status, textvariable=self.sensor_status_var).grid(
            row=0, column=1, sticky="w", padx=6
        )

    def _build_baseline_tab(self) -> None:
        tab = self._new_tab("0. Baseline")
        ttk.Label(
            tab,
            text="현재 기본 설정으로 전체 초기 위치 정합을 실행합니다.",
            style="SubHeader.TLabel",
        ).pack(anchor="w")

        buttons = ttk.Frame(tab)
        buttons.pack(fill="x", pady=8)
        ttk.Button(
            buttons,
            text="전체 측위 시작",
            command=lambda: self.start_full_localization("Baseline"),
        ).pack(side="left", padx=4)
        ttk.Button(
            buttons,
            text="측위 프로세스 중지",
            command=self.stop_localization_stack,
        ).pack(side="left", padx=4)
        ttk.Button(
            buttons,
            text="현재 회차 기록",
            command=lambda: self.add_manual_result("Baseline", "현재 기본값"),
        ).pack(side="left", padx=4)

        self.results["Baseline"] = ResultTable(tab, "Baseline")
        self.results["Baseline"].pack(fill="both", expand=True, pady=6)

    def _build_height_tab(self) -> None:
        tab = self._new_tab("1. SC 높이")
        ttk.Label(
            tab,
            text="z 범위만 바꿔 DB를 새로 생성합니다. 나머지 값은 고정합니다.",
            style="SubHeader.TLabel",
        ).pack(anchor="w")

        presets = ttk.Frame(tab)
        presets.pack(fill="x", pady=6)
        self.height_setting_var = tk.StringVar(value="Z1")
        self.height_zmin_var = tk.DoubleVar(value=-0.35)
        self.height_zmax_var = tk.DoubleVar(value=1.0)

        combo = ttk.Combobox(
            presets,
            textvariable=self.height_setting_var,
            values=("Z1", "Z2", "Z3", "사용자"),
            state="readonly",
            width=10,
        )
        combo.pack(side="left", padx=4)
        combo.bind("<<ComboboxSelected>>", self.apply_height_preset)
        ttk.Label(presets, text="z_min").pack(side="left", padx=(12, 2))
        ttk.Spinbox(
            presets,
            textvariable=self.height_zmin_var,
            from_=-5,
            to=5,
            increment=0.1,
            width=8,
        ).pack(side="left")
        ttk.Label(presets, text="z_max").pack(side="left", padx=(12, 2))
        ttk.Spinbox(
            presets,
            textvariable=self.height_zmax_var,
            from_=-5,
            to=10,
            increment=0.1,
            width=8,
        ).pack(side="left")

        ttk.Button(
            presets,
            text="이 설정으로 DB 생성",
            command=self.build_height_db,
        ).pack(side="left", padx=12)
        ttk.Button(
            presets,
            text="생성 DB로 측위 시작",
            command=lambda: self.start_full_localization("SC 높이"),
        ).pack(side="left", padx=4)
        ttk.Button(
            presets,
            text="현재 회차 기록",
            command=lambda: self.add_manual_result(
                "SC 높이",
                f"{self.height_setting_var.get()} "
                f"[{self.height_zmin_var.get():.2f},{self.height_zmax_var.get():.2f}]",
            ),
        ).pack(side="left", padx=4)

        self.results["SC 높이"] = ResultTable(tab, "SC 높이")
        self.results["SC 높이"].pack(fill="both", expand=True, pady=6)

    def _build_step_tab(self) -> None:
        tab = self._new_tab("2. DB Step")
        ttk.Label(
            tab,
            text="DB 가상 위치 간격만 변경해 후보 밀도와 처리 시간을 비교합니다.",
            style="SubHeader.TLabel",
        ).pack(anchor="w")

        controls = ttk.Frame(tab)
        controls.pack(fill="x", pady=6)
        self.step_setting_var = tk.StringVar(value="S1")
        self.step_var = tk.DoubleVar(value=0.75)
        combo = ttk.Combobox(
            controls,
            textvariable=self.step_setting_var,
            values=("S1", "S2", "S3", "사용자"),
            state="readonly",
            width=10,
        )
        combo.pack(side="left", padx=4)
        combo.bind("<<ComboboxSelected>>", self.apply_step_preset)
        ttk.Label(controls, text="DB step(m)").pack(side="left", padx=(12, 2))
        ttk.Spinbox(
            controls,
            textvariable=self.step_var,
            from_=0.05,
            to=5,
            increment=0.05,
            width=8,
        ).pack(side="left")
        ttk.Button(
            controls, text="DB 생성", command=self.build_step_db
        ).pack(side="left", padx=12)
        ttk.Button(
            controls,
            text="생성 DB로 측위 시작",
            command=lambda: self.start_full_localization("DB Step"),
        ).pack(side="left", padx=4)
        ttk.Button(
            controls,
            text="현재 회차 기록",
            command=lambda: self.add_manual_result(
                "DB Step",
                f"{self.step_setting_var.get()} step={self.step_var.get():.2f}",
            ),
        ).pack(side="left", padx=4)

        self.results["DB Step"] = ResultTable(tab, "DB Step")
        self.results["DB Step"].pack(fill="both", expand=True, pady=6)

    def _build_icp_tab(self) -> None:
        tab = self._new_tab("3. ICP 비교")
        ttk.Label(
            tab,
            text=(
                "기존 Single ICP를 직접 실행하거나, SC 후보를 받아 "
                "Coarse → Fine 순서로 자동 실행합니다."
            ),
            style="SubHeader.TLabel",
        ).pack(anchor="w")

        body = ttk.Frame(tab)
        body.pack(fill="x", pady=6)

        single = ttk.LabelFrame(body, text="Single ICP", padding=8)
        single.pack(side="left", fill="both", expand=True, padx=(0, 4))
        self._spin_row(single, 0, "Map voxel", self.icp_map_voxel, 0.01, 5, 0.05)
        self._spin_row(single, 1, "Cloud voxel", self.icp_cloud_voxel, 0.01, 5, 0.05)
        self._spin_row(single, 2, "Iteration", self.icp_iterations, 1, 1000, 1)
        self._spin_row(single, 3, "Correspondence", self.icp_corr, 0.01, 10, 0.05)
        self._spin_row(single, 4, "Fitness threshold", self.icp_fitness, 0.0001, 10, 0.01)
        self._spin_row(single, 5, "Converged count", self.icp_count, 1, 1000, 1)

        coarse = ttk.LabelFrame(body, text="Coarse ICP", padding=8)
        coarse.pack(side="left", fill="both", expand=True, padx=4)
        self._spin_row(coarse, 0, "Map voxel", self.coarse_map_voxel, 0.01, 5, 0.05)
        self._spin_row(coarse, 1, "Cloud voxel", self.coarse_cloud_voxel, 0.01, 5, 0.05)
        self._spin_row(coarse, 2, "Iteration", self.coarse_iterations, 1, 1000, 1)
        self._spin_row(coarse, 3, "Correspondence", self.coarse_corr, 0.01, 10, 0.05)
        self._spin_row(coarse, 4, "Fitness threshold", self.coarse_fitness, 0.0001, 10, 0.01)

        fine = ttk.LabelFrame(body, text="Fine ICP", padding=8)
        fine.pack(side="left", fill="both", expand=True, padx=(4, 0))
        self._spin_row(fine, 0, "Map voxel", self.fine_map_voxel, 0.01, 5, 0.05)
        self._spin_row(fine, 1, "Cloud voxel", self.fine_cloud_voxel, 0.01, 5, 0.05)
        self._spin_row(fine, 2, "Iteration", self.fine_iterations, 1, 1000, 1)
        self._spin_row(fine, 3, "Correspondence", self.fine_corr, 0.01, 10, 0.05)
        self._spin_row(fine, 4, "Fitness threshold", self.fine_fitness, 0.0001, 10, 0.01)

        initial = ttk.LabelFrame(tab, text="수동 초기값(필요 시)", padding=8)
        initial.pack(fill="x", pady=5)
        for label, var in (
            ("x", self.initial_x_var),
            ("y", self.initial_y_var),
            ("z", self.initial_z_var),
            ("yaw(rad)", self.initial_yaw_var),
        ):
            ttk.Label(initial, text=label).pack(side="left", padx=(6, 2))
            ttk.Entry(initial, textvariable=var, width=10).pack(side="left")

        buttons = ttk.Frame(tab)
        buttons.pack(fill="x", pady=5)
        ttk.Button(
            buttons,
            text="Single ICP 실행",
            command=lambda: self.start_single_icp("ICP 비교", "I1 Single"),
        ).pack(side="left", padx=4)
        ttk.Button(
            buttons,
            text="SC 후보 자동 캡처 → Coarse → Fine",
            command=self.start_coarse_to_fine,
        ).pack(side="left", padx=4)
        ttk.Button(
            buttons,
            text="ICP 프로세스 중지",
            command=lambda: (
                self.process_manager.stop("icp_single"),
                self.process_manager.stop("icp_coarse"),
                self.process_manager.stop("icp_fine"),
                self.process_manager.stop("sc_capture"),
            ),
        ).pack(side="left", padx=4)
        ttk.Button(
            buttons,
            text="현재 회차 기록",
            command=lambda: self.add_manual_result("ICP 비교", "수동 기록"),
        ).pack(side="left", padx=4)

        self.results["ICP 비교"] = ResultTable(tab, "ICP 비교")
        self.results["ICP 비교"].pack(fill="both", expand=True, pady=6)

    def _build_accum_tab(self) -> None:
        tab = self._new_tab("4. 누적 프레임")
        ttk.Label(
            tab,
            text=(
                "현재 저장소의 sc_localizer 누적 프레임은 바로 변경할 수 있습니다. "
                "ICP 자체의 누적 source 사용은 별도 패치 전까지 SC 후보 품질 비교용입니다."
            ),
            style="SubHeader.TLabel",
        ).pack(anchor="w")

        controls = ttk.Frame(tab)
        controls.pack(fill="x", pady=8)
        self.accum_test_var = tk.IntVar(value=1)
        ttk.Label(controls, text="누적 프레임").pack(side="left")
        ttk.Combobox(
            controls,
            textvariable=self.accum_test_var,
            values=(1, 5, 10, 20),
            width=8,
        ).pack(side="left", padx=6)
        ttk.Button(
            controls,
            text="이 프레임 수로 측위 시작",
            command=self.start_accum_test,
        ).pack(side="left", padx=4)
        ttk.Button(
            controls,
            text="현재 회차 기록",
            command=lambda: self.add_manual_result(
                "누적 프레임", f"{self.accum_test_var.get()} frames"
            ),
        ).pack(side="left", padx=4)

        self.results["누적 프레임"] = ResultTable(tab, "누적 프레임")
        self.results["누적 프레임"].pack(fill="both", expand=True, pady=6)

    def _build_timeout_tab(self) -> None:
        tab = self._new_tab("5. Timeout·후보")
        ttk.Label(
            tab,
            text="SC 후보 개수, 후보당 대기 시간, ICP 성공 횟수를 한 설정씩 비교합니다.",
            style="SubHeader.TLabel",
        ).pack(anchor="w")

        controls = ttk.Frame(tab)
        controls.pack(fill="x", pady=8)
        self.timeout_preset_var = tk.StringVar(value="T1")
        combo = ttk.Combobox(
            controls,
            textvariable=self.timeout_preset_var,
            values=("T1", "T2", "T3", "C1", "C2", "C3", "사용자"),
            state="readonly",
            width=10,
        )
        combo.pack(side="left", padx=4)
        combo.bind("<<ComboboxSelected>>", self.apply_timeout_preset)

        ttk.Label(controls, text="성공 횟수").pack(side="left", padx=(12, 2))
        ttk.Spinbox(
            controls, textvariable=self.icp_count, from_=1, to=1000, width=7
        ).pack(side="left")
        ttk.Label(controls, text="timeout").pack(side="left", padx=(12, 2))
        ttk.Spinbox(
            controls,
            textvariable=self.common_timeout,
            from_=0.5,
            to=120,
            increment=0.5,
            width=7,
        ).pack(side="left")
        ttk.Label(controls, text="topk").pack(side="left", padx=(12, 2))
        ttk.Spinbox(
            controls, textvariable=self.common_topk, from_=1, to=500, width=7
        ).pack(side="left")
        ttk.Label(controls, text="후보 수").pack(side="left", padx=(12, 2))
        ttk.Spinbox(
            controls,
            textvariable=self.common_candidates,
            from_=1,
            to=100,
            width=7,
        ).pack(side="left")

        ttk.Button(
            controls,
            text="이 설정으로 측위 시작",
            command=lambda: self.start_full_localization("Timeout·후보"),
        ).pack(side="left", padx=12)
        ttk.Button(
            controls,
            text="현재 회차 기록",
            command=lambda: self.add_manual_result(
                "Timeout·후보", self.timeout_preset_var.get()
            ),
        ).pack(side="left", padx=4)

        self.results["Timeout·후보"] = ResultTable(tab, "Timeout·후보")
        self.results["Timeout·후보"].pack(fill="both", expand=True, pady=6)

    def _build_final_tab(self) -> None:
        tab = self._new_tab("6. 최종 통합")
        ttk.Label(
            tab,
            text="현재 GUI에 입력된 최종값으로 측위를 반복 실행하고 결과를 저장합니다.",
            style="SubHeader.TLabel",
        ).pack(anchor="w")

        controls = ttk.Frame(tab)
        controls.pack(fill="x", pady=8)
        self.final_repeat_var = tk.IntVar(value=10)
        ttk.Label(controls, text="반복 횟수").pack(side="left")
        ttk.Spinbox(
            controls,
            textvariable=self.final_repeat_var,
            from_=1,
            to=100,
            width=8,
        ).pack(side="left", padx=6)
        ttk.Button(
            controls,
            text="현재 설정으로 1회 시작",
            command=lambda: self.start_full_localization("최종 통합"),
        ).pack(side="left", padx=4)
        ttk.Button(
            controls,
            text="현재 회차 기록",
            command=lambda: self.add_manual_result("최종 통합", "Final"),
        ).pack(side="left", padx=4)
        ttk.Button(
            controls,
            text="모든 결과 CSV 저장",
            command=self.export_all_results,
        ).pack(side="right", padx=4)

        self.results["최종 통합"] = ResultTable(tab, "최종 통합")
        self.results["최종 통합"].pack(fill="both", expand=True, pady=6)

    # ------------------------------------------------------------------
    # Environment and logging
    # ------------------------------------------------------------------
    def enqueue_log(self, message: str) -> None:
        self.log_queue.put(f"{now_text()} {message}")

    def _drain_log_queue(self) -> None:
        try:
            while True:
                msg = self.log_queue.get_nowait()
                self.log_text.configure(state="normal")
                self.log_text.insert("end", msg + "\n")
                self.log_text.see("end")
                self.log_text.configure(state="disabled")
        except queue.Empty:
            pass
        self.after(100, self._drain_log_queue)

    def detect_environment(self) -> None:
        ros2 = shutil.which("ros2")
        ws = Path(self.ws_var.get()).expanduser()
        install_setup = ws / "install" / "setup.bash"

        checks = {
            "운영체제": platform.platform(),
            "ros2 명령": ros2 or "없음",
            "ROS setup": str(Path(self.ros_setup_var.get()).expanduser()),
            "워크스페이스": str(ws),
            "install/setup.bash": str(install_setup),
        }
        for key, value in checks.items():
            self.enqueue_log(f"[환경] {key}: {value}")

        if ros2 and install_setup.exists():
            self.status_var.set("Jetson/ROS 실행 가능")
        else:
            self.status_var.set("ROS 미검출: 미리보기 권장")
            self.preview_var.set(True)

    def ros_shell(self, command: str) -> str:
        ros_setup = expand_path(self.ros_setup_var.get())
        ws_setup = str(Path(self.ws_var.get()).expanduser() / "install" / "setup.bash")
        return (
            "set -o pipefail; "
            f"source {shell_quote(ros_setup)}; "
            f"source {shell_quote(ws_setup)}; "
            f"exec {command}"
        )

    def execute(
        self,
        name: str,
        command: str,
        managed: bool = True,
        on_exit: Optional[Callable[[int], None]] = None,
    ) -> None:
        self.last_command = command
        self.enqueue_log(f"[명령] {command}")

        if self.preview_var.get():
            self.enqueue_log("[미리보기] 실제 실행하지 않았습니다.")
            return

        try:
            if managed:
                self.process_manager.start(
                    name,
                    self.ros_shell(command),
                    cwd=expand_path(self.ws_var.get()),
                    on_exit=on_exit,
                )
            else:
                subprocess.Popen(
                    ["bash", "-lc", self.ros_shell(command)],
                    cwd=expand_path(self.ws_var.get()),
                )
        except Exception as exc:
            self.enqueue_log(f"[실행 오류] {exc}")
            messagebox.showerror("실행 오류", str(exc))

    def capture_command(
        self,
        command: str,
        timeout: float = 30.0,
    ) -> Tuple[int, str]:
        if self.preview_var.get():
            self.enqueue_log(f"[미리보기 캡처 명령] {command}")
            return 0, ""

        full = self.ros_shell(command)
        proc = subprocess.run(
            ["bash", "-lc", full],
            cwd=expand_path(self.ws_var.get()),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
            check=False,
        )
        self.enqueue_log(f"[캡처 결과 code={proc.returncode}]\n{proc.stdout}")
        return proc.returncode, proc.stdout

    # ------------------------------------------------------------------
    # Command builders
    # ------------------------------------------------------------------
    def sc_db_command(
        self,
        out_path: str,
        step: float,
        z_min: float,
        z_max: float,
    ) -> str:
        return " ".join(
            [
                "ros2 run alm_navigation sc_build_db.py",
                "--pcd", shell_quote(expand_path(self.map_var.get())),
                "--out", shell_quote(expand_path(out_path)),
                "--step", str(step),
                "--keyframe-z", str(self.db_keyframe_z.get()),
                "--num-ring", str(self.db_num_ring.get()),
                "--num-sector", str(self.db_num_sector.get()),
                "--max-radius", str(self.db_radius.get()),
                "--z-min", str(z_min),
                "--z-max", str(z_max),
                "--topk", str(self.common_topk.get()),
                "--selftest", str(self.db_selftest.get()),
            ]
        )

    def sc_localizer_command(
        self,
        db_path: Optional[str] = None,
        accum_frames: Optional[int] = None,
        topk: Optional[int] = None,
        candidates: Optional[int] = None,
        timeout_sec: Optional[float] = None,
    ) -> str:
        return " ".join(
            [
                "ros2 run alm_navigation sc_localizer.py --ros-args",
                "-p", f"db_path:={shell_quote(expand_path(db_path or self.db_var.get()))}",
                "-p", "lidar_topic:=/livox/lidar",
                "-p", f"accum_frames:={accum_frames or self.common_accum_frames.get()}",
                "-p", f"topk:={topk or self.common_topk.get()}",
                "-p", f"max_candidates:={candidates or self.common_candidates.get()}",
                "-p", f"icp_wait_sec:={timeout_sec or self.common_timeout.get()}",
            ]
        )

    def icp_command(
        self,
        initial: Optional[Tuple[float, float, float, float]] = None,
        map_voxel: Optional[float] = None,
        cloud_voxel: Optional[float] = None,
        iterations: Optional[int] = None,
        correspondence: Optional[float] = None,
        fitness: Optional[float] = None,
        converged_count: Optional[int] = None,
    ) -> str:
        x, y, z, yaw = initial or (
            self.initial_x_var.get(),
            self.initial_y_var.get(),
            self.initial_z_var.get(),
            self.initial_yaw_var.get(),
        )
        return " ".join(
            [
                "ros2 run icp_relocalization icp_node --ros-args",
                "-p", f"initial_x:={x}",
                "-p", f"initial_y:={y}",
                "-p", f"initial_z:={z}",
                "-p", f"initial_a:={yaw}",
                "-p", f"map_voxel_leaf_size:={map_voxel if map_voxel is not None else self.icp_map_voxel.get()}",
                "-p", f"cloud_voxel_leaf_size:={cloud_voxel if cloud_voxel is not None else self.icp_cloud_voxel.get()}",
                "-p", "map_frame_id:=map",
                "-p", f"solver_max_iter:={iterations if iterations is not None else self.icp_iterations.get()}",
                "-p", f"max_correspondence_distance:={correspondence if correspondence is not None else self.icp_corr.get()}",
                "-p", f"RANSAC_outlier_rejection_threshold:={self.icp_ransac.get()}",
                "-p", f"map_path:={shell_quote(expand_path(self.map_var.get()))}",
                "-p", f"fitness_score_thre:={fitness if fitness is not None else self.icp_fitness.get()}",
                "-p", f"converged_count_thre:={converged_count if converged_count is not None else self.icp_count.get()}",
                "-p", "pcl_type:=pointcloud",
                "-r", "/pointcloud2:=/livox/lidar",
            ]
        )

    def transform_command(self) -> str:
        return (
            "ros2 run icp_relocalization transform_publisher --ros-args "
            "-p map_frame_id:=map -p odom_frame_id:=odom"
        )

    def fastlio_command(self) -> str:
        return (
            "ros2 run fast_lio fastlio_mapping --ros-args "
            f"--params-file {shell_quote(expand_path(self.fastlio_config_var.get()))}"
        )

    # ------------------------------------------------------------------
    # Actions
    # ------------------------------------------------------------------
    def start_sensor(self) -> None:
        self.execute(
            "sensor",
            "ros2 launch alm_sensors lidar.launch.py",
            on_exit=lambda _: self.sensor_status_var.set("센서 종료"),
        )
        if not self.preview_var.get():
            self.sensor_status_var.set("센서 실행 중")

    def start_rviz(self) -> None:
        ws = Path(self.ws_var.get()).expanduser()
        rviz_config = (
            ws
            / "install"
            / "alm_navigation"
            / "share"
            / "alm_navigation"
            / "rviz"
            / "localization.rviz"
        )
        command = f"rviz2 -d {shell_quote(str(rviz_config))}"
        self.execute("rviz", command)

    def stop_all(self) -> None:
        self.process_manager.stop_all()
        self.sensor_status_var.set("센서 미실행")

    def stop_localization_stack(self) -> None:
        for name in (
            "sc_localizer",
            "icp_node",
            "fastlio",
            "transform_publisher",
        ):
            self.process_manager.stop(name)

    def start_full_localization(self, experiment: str) -> None:
        if self.auto_start_sensor_var.get() and not self.process_manager.is_running("sensor"):
            self.start_sensor()

        self.stop_localization_stack()
        self.experiment_start_time[experiment] = time.monotonic()

        self.execute("transform_publisher", self.transform_command())
        if self.preview_var.get():
            self.execute("icp_node", self.icp_command())
            self.execute("fastlio", self.fastlio_command())
            self.execute("sc_localizer", self.sc_localizer_command())
            return

        def delayed_start() -> None:
            time.sleep(3.0)
            try:
                self.execute("icp_node", self.icp_command())
                self.execute("fastlio", self.fastlio_command())
                self.execute("sc_localizer", self.sc_localizer_command())
            except Exception as exc:
                self.enqueue_log(f"[측위 시작 오류] {exc}")

        threading.Thread(target=delayed_start, daemon=True).start()

    def apply_height_preset(self, _event: object = None) -> None:
        preset = self.height_setting_var.get()
        values = {
            "Z1": (-0.35, 1.0),
            "Z2": (-0.2, 0.7),
            "Z3": (-0.2, 0.5),
        }
        if preset in values:
            z_min, z_max = values[preset]
            self.height_zmin_var.set(z_min)
            self.height_zmax_var.set(z_max)

    def build_height_db(self) -> None:
        setting = self.height_setting_var.get()
        base = Path(self.db_var.get()).expanduser()
        out = base.with_name(
            f"{base.stem}_{setting}_z{self.height_zmin_var.get():.1f}_"
            f"{self.height_zmax_var.get():.1f}{base.suffix}"
        )
        self.db_var.set(str(out))
        command = self.sc_db_command(
            str(out),
            self.step_var.get() if hasattr(self, "step_var") else 0.75,
            self.height_zmin_var.get(),
            self.height_zmax_var.get(),
        )
        self.execute(f"db_height_{setting}", command)

    def apply_step_preset(self, _event: object = None) -> None:
        values = {"S1": 0.75, "S2": 0.50, "S3": 0.35}
        if self.step_setting_var.get() in values:
            self.step_var.set(values[self.step_setting_var.get()])

    def build_step_db(self) -> None:
        setting = self.step_setting_var.get()
        base = Path(self.db_var.get()).expanduser()
        out = base.with_name(f"{base.stem}_{setting}_step{self.step_var.get():.2f}{base.suffix}")
        self.db_var.set(str(out))
        z_min = self.height_zmin_var.get() if hasattr(self, "height_zmin_var") else -0.35
        z_max = self.height_zmax_var.get() if hasattr(self, "height_zmax_var") else 1.0
        command = self.sc_db_command(str(out), self.step_var.get(), z_min, z_max)
        self.execute(f"db_step_{setting}", command)

    def start_single_icp(self, experiment: str, setting: str) -> None:
        self.experiment_start_time[experiment] = time.monotonic()
        self.execute("icp_single", self.icp_command())
        self.enqueue_log(f"[실험 설정] {experiment}: {setting}")

    def start_accum_test(self) -> None:
        previous = self.common_accum_frames.get()
        self.common_accum_frames.set(self.accum_test_var.get())
        try:
            self.start_full_localization("누적 프레임")
        finally:
            # 화면에는 선택값을 유지하되 공통값도 실험값으로 맞춰 둡니다.
            self.enqueue_log(
                f"[누적 프레임] {previous} → {self.accum_test_var.get()}"
            )

    def apply_timeout_preset(self, _event: object = None) -> None:
        preset = self.timeout_preset_var.get()
        values = {
            # T1 = "현재 기준". localization.launch.py 의 converged_count_thre=10 과 동일.
            "T1": (10, 12.0, 25, 5),
            "T2": (5, 5.0, 25, 5),
            "T3": (3, 4.0, 25, 5),
            "C1": (3, 4.0, 25, 5),
            "C2": (3, 4.0, 50, 5),
            "C3": (3, 4.0, 50, 10),
        }
        if preset in values:
            count, timeout, topk, candidates = values[preset]
            self.icp_count.set(count)
            self.common_timeout.set(timeout)
            self.common_topk.set(topk)
            self.common_candidates.set(candidates)

    # ------------------------------------------------------------------
    # Coarse-to-Fine automation
    # ------------------------------------------------------------------
    def start_coarse_to_fine(self) -> None:
        if self.preview_var.get():
            self.enqueue_log("[Coarse-to-Fine 미리보기]")
            self.enqueue_log("1) sc_localizer로 /initialpose 캡처")
            self.enqueue_log("2) Coarse icp_node 실행 및 /icp_result 캡처")
            self.enqueue_log("3) Fine icp_node 실행 및 최종 /icp_result 캡처")
            self.enqueue_log(
                "[Coarse 명령] "
                + self.icp_command(
                    map_voxel=self.coarse_map_voxel.get(),
                    cloud_voxel=self.coarse_cloud_voxel.get(),
                    iterations=self.coarse_iterations.get(),
                    correspondence=self.coarse_corr.get(),
                    fitness=self.coarse_fitness.get(),
                    converged_count=1,
                )
            )
            self.enqueue_log(
                "[Fine 명령] "
                + self.icp_command(
                    map_voxel=self.fine_map_voxel.get(),
                    cloud_voxel=self.fine_cloud_voxel.get(),
                    iterations=self.fine_iterations.get(),
                    correspondence=self.fine_corr.get(),
                    fitness=self.fine_fitness.get(),
                    converged_count=1,
                )
            )
            return

        threading.Thread(target=self._coarse_to_fine_worker, daemon=True).start()

    def _coarse_to_fine_worker(self) -> None:
        experiment = "ICP 비교"
        setting = "I3 Coarse-to-Fine"
        start = time.monotonic()

        try:
            self.process_manager.stop("sc_capture")
            self.process_manager.stop("icp_coarse")
            self.process_manager.stop("icp_fine")

            self.execute(
                "sc_capture",
                self.sc_localizer_command(
                    candidates=1,
                    timeout_sec=max(30.0, self.common_timeout.get()),
                ),
            )
            time.sleep(0.8)

            code, initial_output = self.capture_command(
                "timeout 25s ros2 topic echo --once /initialpose",
                timeout=30,
            )
            self.process_manager.stop("sc_capture")
            if code != 0:
                raise RuntimeError("SC initialpose 캡처 실패")
            initial = self.parse_pose_message(initial_output)
            self.enqueue_log(f"[SC 초기 후보] {initial}")

            coarse_command = self.icp_command(
                initial=initial,
                map_voxel=self.coarse_map_voxel.get(),
                cloud_voxel=self.coarse_cloud_voxel.get(),
                iterations=self.coarse_iterations.get(),
                correspondence=self.coarse_corr.get(),
                fitness=self.coarse_fitness.get(),
                converged_count=1,
            )
            self.execute("icp_coarse", coarse_command)
            time.sleep(0.5)
            code, coarse_output = self.capture_command(
                "timeout 20s ros2 topic echo --once /icp_result",
                timeout=25,
            )
            self.process_manager.stop("icp_coarse")
            if code != 0:
                raise RuntimeError("Coarse ICP 결과 캡처 실패")
            coarse_pose = self.parse_pose_message(coarse_output)
            self.enqueue_log(f"[Coarse 결과] {coarse_pose}")

            fine_command = self.icp_command(
                initial=coarse_pose,
                map_voxel=self.fine_map_voxel.get(),
                cloud_voxel=self.fine_cloud_voxel.get(),
                iterations=self.fine_iterations.get(),
                correspondence=self.fine_corr.get(),
                fitness=self.fine_fitness.get(),
                converged_count=1,
            )
            self.execute("icp_fine", fine_command)
            time.sleep(0.5)
            code, fine_output = self.capture_command(
                "timeout 20s ros2 topic echo --once /icp_result",
                timeout=25,
            )
            self.process_manager.stop("icp_fine")
            if code != 0:
                raise RuntimeError("Fine ICP 결과 캡처 실패")
            fine_pose = self.parse_pose_message(fine_output)
            self.enqueue_log(f"[Fine 결과] {fine_pose}")

            elapsed = time.monotonic() - start
            self.results[experiment].add(
                ExperimentResult(
                    timestamp=now_text(),
                    experiment=experiment,
                    setting=setting,
                    repeat=self.next_repeat(experiment),
                    elapsed_sec=f"{elapsed:.3f}",
                    auto_status="Coarse/Fine 성공",
                    note=(
                        f"initial={tuple(round(v, 3) for v in initial)}, "
                        f"coarse={tuple(round(v, 3) for v in coarse_pose)}, "
                        f"fine={tuple(round(v, 3) for v in fine_pose)}"
                    ),
                )
            )
        except Exception as exc:
            elapsed = time.monotonic() - start
            self.enqueue_log(f"[Coarse-to-Fine 실패] {exc}")
            self.results[experiment].add(
                ExperimentResult(
                    timestamp=now_text(),
                    experiment=experiment,
                    setting=setting,
                    repeat=self.next_repeat(experiment),
                    elapsed_sec=f"{elapsed:.3f}",
                    auto_status="실패",
                    note=str(exc),
                )
            )

    @staticmethod
    def parse_pose_message(text: str) -> Tuple[float, float, float, float]:
        """ros2 topic echo PoseWithCovarianceStamped 출력에서 x,y,z,yaw를 추출."""
        if yaml is None:
            raise RuntimeError("python3-yaml이 필요합니다: sudo apt install python3-yaml")

        docs = [doc for doc in yaml.safe_load_all(text) if isinstance(doc, dict)]
        if not docs:
            raise ValueError("ROS pose YAML을 해석할 수 없습니다.")

        data = docs[-1]
        pose_container = data.get("pose", {})
        if "pose" in pose_container:
            pose = pose_container["pose"]
        else:
            pose = pose_container

        position = pose["position"]
        orientation = pose["orientation"]
        x = float(position["x"])
        y = float(position["y"])
        z = float(position["z"])
        qx = float(orientation.get("x", 0.0))
        qy = float(orientation.get("y", 0.0))
        qz = float(orientation.get("z", 0.0))
        qw = float(orientation.get("w", 1.0))

        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return x, y, z, yaw

    # ------------------------------------------------------------------
    # Results and config
    # ------------------------------------------------------------------
    def next_repeat(self, experiment: str) -> int:
        value = self.current_repeat.get(experiment, 0) + 1
        self.current_repeat[experiment] = value
        return value

    def add_manual_result(self, experiment: str, setting: str) -> None:
        elapsed = ""
        if experiment in self.experiment_start_time:
            elapsed = f"{time.monotonic() - self.experiment_start_time[experiment]:.3f}"
        self.results[experiment].add(
            ExperimentResult(
                timestamp=now_text(),
                experiment=experiment,
                setting=setting,
                repeat=self.next_repeat(experiment),
                elapsed_sec=elapsed,
                auto_status="수동 기록",
            )
        )

    def collect_config(self) -> dict:
        return {
            "workspace": self.ws_var.get(),
            "map_pcd": self.map_var.get(),
            "sc_db": self.db_var.get(),
            "fastlio_config": self.fastlio_config_var.get(),
            "output_dir": self.output_var.get(),
            "branch": self.branch_var.get(),
            "ros_setup": self.ros_setup_var.get(),
            "preview": self.preview_var.get(),
            "scan_context": {
                "accum_frames": self.common_accum_frames.get(),
                "topk": self.common_topk.get(),
                "max_candidates": self.common_candidates.get(),
                "icp_wait_sec": self.common_timeout.get(),
                "num_ring": self.db_num_ring.get(),
                "num_sector": self.db_num_sector.get(),
                "max_radius": self.db_radius.get(),
                "keyframe_z": self.db_keyframe_z.get(),
            },
            "single_icp": {
                "map_voxel": self.icp_map_voxel.get(),
                "cloud_voxel": self.icp_cloud_voxel.get(),
                "iterations": self.icp_iterations.get(),
                "correspondence": self.icp_corr.get(),
                "fitness": self.icp_fitness.get(),
                "converged_count": self.icp_count.get(),
            },
            "coarse_icp": {
                "map_voxel": self.coarse_map_voxel.get(),
                "cloud_voxel": self.coarse_cloud_voxel.get(),
                "iterations": self.coarse_iterations.get(),
                "correspondence": self.coarse_corr.get(),
                "fitness": self.coarse_fitness.get(),
            },
            "fine_icp": {
                "map_voxel": self.fine_map_voxel.get(),
                "cloud_voxel": self.fine_cloud_voxel.get(),
                "iterations": self.fine_iterations.get(),
                "correspondence": self.fine_corr.get(),
                "fitness": self.fine_fitness.get(),
            },
        }

    def save_gui_config(self) -> None:
        path = filedialog.asksaveasfilename(
            title="GUI 설정 저장",
            defaultextension=".json",
            filetypes=[("JSON", "*.json")],
        )
        if not path:
            return
        Path(path).write_text(
            json.dumps(self.collect_config(), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        self.enqueue_log(f"[설정 저장] {path}")

    def load_gui_config(self) -> None:
        path = filedialog.askopenfilename(
            title="GUI 설정 불러오기",
            filetypes=[("JSON", "*.json")],
        )
        if not path:
            return
        data = json.loads(Path(path).read_text(encoding="utf-8"))
        self.ws_var.set(data.get("workspace", self.ws_var.get()))
        self.map_var.set(data.get("map_pcd", self.map_var.get()))
        self.db_var.set(data.get("sc_db", self.db_var.get()))
        self.fastlio_config_var.set(
            data.get("fastlio_config", self.fastlio_config_var.get())
        )
        self.output_var.set(data.get("output_dir", self.output_var.get()))
        self.branch_var.set(data.get("branch", self.branch_var.get()))
        self.ros_setup_var.set(data.get("ros_setup", self.ros_setup_var.get()))

        sc = data.get("scan_context", {})
        self.common_accum_frames.set(sc.get("accum_frames", self.common_accum_frames.get()))
        self.common_topk.set(sc.get("topk", self.common_topk.get()))
        self.common_candidates.set(
            sc.get("max_candidates", self.common_candidates.get())
        )
        self.common_timeout.set(sc.get("icp_wait_sec", self.common_timeout.get()))
        self.enqueue_log(f"[설정 불러오기] {path}")

    def export_all_results(self) -> None:
        output = Path(self.output_var.get()).expanduser() / f"gui_export_{run_id()}"
        output.mkdir(parents=True, exist_ok=True)

        config_path = output / "parameters.json"
        config_path.write_text(
            json.dumps(self.collect_config(), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

        all_rows: List[ExperimentResult] = []
        for table in self.results.values():
            all_rows.extend(table.export_rows())

        csv_path = output / "results.csv"
        fieldnames = list(ExperimentResult.__dataclass_fields__.keys())
        with csv_path.open("w", newline="", encoding="utf-8-sig") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in all_rows:
                writer.writerow(asdict(row))

        log_path = output / "gui_log.txt"
        log_path.write_text(
            self.log_text.get("1.0", "end"),
            encoding="utf-8",
        )

        self.enqueue_log(f"[결과 저장] {output}")
        messagebox.showinfo("저장 완료", f"결과를 저장했습니다.\n{output}")

    def on_close(self) -> None:
        if messagebox.askokcancel(
            "종료",
            "실행 중인 ROS 프로세스를 모두 종료하고 GUI를 닫겠습니까?",
        ):
            self.stop_all()
            self.destroy()


def main() -> None:
    app = ALMExperimentGUI()
    app.mainloop()


if __name__ == "__main__":
    main()
