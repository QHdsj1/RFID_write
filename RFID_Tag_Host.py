# -*- coding: utf-8 -*-
"""
RFID 标签自动读写上位机
协议: 波特率 38400, 8 数据位, 1 停止位, 无校验 (8N1)
校验: ADD8 (前 N 字节十六进制求和取低 8 位)

帧格式:
  写入: 53 57 30 30 30 30 30 31 + 4字节数据 + ADD8(前12字节) + 03
  读入: 45 52 30 30 30 30 30 31 B8 03
  应答: 30 + 4字节数据 + ADD8(前5字节) + 03
  写成功: 30 30 03   写失败: 34 34 03   标签离开: 35 35 03

依赖: pip install pyserial
"""

import os
import glob
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
import time
import datetime

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    serial = None
    list_ports = None

# ---------------- 协议常量 ----------------
BAUD = 38400
WRITE_HEAD = bytes([0x53, 0x57, 0x30, 0x30, 0x30, 0x30, 0x30, 0x31])   # "SW000001"
READ_CMD   = bytes([0x45, 0x52, 0x30, 0x30, 0x30, 0x30, 0x30, 0x31, 0xB8, 0x03])  # "ER000001"
TAIL = 0x03


def add8(data: bytes) -> int:
    """ADD8 校验: 十六进制求和取低 8 位"""
    return sum(data) & 0xFF


def build_write_frame(data4: bytes) -> bytes:
    body = WRITE_HEAD + data4
    return body + bytes([add8(body), TAIL])


def parse_tag_info(data4: bytes) -> str:
    """把 4 字节应答数据翻译成可读信息"""
    num = int.from_bytes(data4, 'big')
    if data4 == bytes([0x11] * 4):
        return "四驱落地式护栏机(右)  数据: 11 11 11 11"
    if data4 == bytes([0x22] * 4):
        return "四驱落地式护栏机(充电桩)  数据: 22 22 22 22"
    if data4 == bytes([0x33] * 4):
        return "四驱落地式护栏机(左)  数据: 33 33 33 33"
    if data4[0] == 0x4D:
        return f"{data4[0]:#04X} 号充电桩 (4D=主机)" if num == 0x4D else f"{num & 0xFFFFFF} 号充电桩"
    if data4[0] == 0x46:
        return f"{num & 0xFFFFFF} 号维修点 (0x46='F')"
    return f"{num} 号标签 (数据: {data4.hex(' ').upper()})"


# ---------------- 主窗口 ----------------
class TagHostApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("RFID 标签自动读写上位机  (38400, 8N1)")
        self.geometry("1100x720")
        self.resizable(True, True)
        self.minsize(900, 600)

        self.ser = None
        self.rx_thread = None
        self.running = False          # 串口线程运行标志
        self.auto_read_on = False     # 自动轮询标志
        self.rx_buf = bytearray()
        self.ser_lock = threading.Lock()   # 保护串口写操作

        self._build_ui()

    # ---------- 界面 ----------
    def _build_ui(self):
        # ===== 串口设置区 =====
        frm_conn = ttk.LabelFrame(self, text=" 串口设置 ")
        frm_conn.pack(fill='x', padx=8, pady=6)

        ttk.Label(frm_conn, text="串口:").grid(row=0, column=0, padx=4, pady=6, sticky='e')
        self.cmb_port = ttk.Combobox(frm_conn, width=20, state='readonly')
        self.cmb_port.grid(row=0, column=1, padx=4)

        # 手动输入开关
        self.var_manual_port = tk.BooleanVar(value=False)
        self.chk_manual = ttk.Checkbutton(
            frm_conn, text="手动输入",
            variable=self.var_manual_port,
            command=self._toggle_manual_port
        )
        self.chk_manual.grid(row=0, column=2, padx=4)

        ttk.Button(frm_conn, text="刷新", width=6,
                   command=self.refresh_ports).grid(row=0, column=3, padx=2)

        ttk.Label(frm_conn, text="波特率:").grid(row=0, column=4, padx=4, sticky='e')
        self.cmb_baud = ttk.Combobox(
            frm_conn, width=8,
            values=["38400", "9600", "19200", "57600", "115200"]
        )
        self.cmb_baud.set("38400")
        self.cmb_baud.grid(row=0, column=5, padx=4)

        ttk.Label(frm_conn, text="数据格式:").grid(row=0, column=6, padx=4, sticky='e')
        ttk.Label(frm_conn, text="8,1,N").grid(row=0, column=7, padx=4, sticky='w')

        self.btn_open = ttk.Button(frm_conn, text="打开串口", command=self.toggle_serial)
        self.btn_open.grid(row=0, column=8, padx=10)

        self.refresh_ports()
        self._build_ui_rest()

    def _build_ui_rest(self):
        # ===== 自动读取区 =====
        frm_read = ttk.LabelFrame(self, text=" 自动读取标签 ")
        frm_read.pack(fill='x', padx=8, pady=6)

        ttk.Label(frm_read, text="轮询间隔(ms):").grid(row=0, column=0, padx=4, pady=8, sticky='e')
        self.spin_interval = ttk.Spinbox(frm_read, from_=50, to=60000, width=8)
        self.spin_interval.set(500)
        self.spin_interval.grid(row=0, column=1, padx=4)

        self.btn_auto = ttk.Button(frm_read, text="开始自动读取", command=self.toggle_auto_read)
        self.btn_auto.grid(row=0, column=2, padx=8)

        self.btn_read_once = ttk.Button(frm_read, text="读取一次", command=self.read_once)
        self.btn_read_once.grid(row=0, column=3, padx=4)

        self.lbl_last = ttk.Label(
            frm_read, text="当前标签: 无", foreground="#0066CC",
            font=("", 11, "bold")
        )
        self.lbl_last.grid(row=0, column=4, padx=16, sticky='w')

        # ===== 写入区 =====
        self._build_ui_write()

    def _build_ui_write(self):
        frm_write = ttk.LabelFrame(
            self,
            text=" 写入标签 (固定头 53 57 30 30 30 30 30 31 + 4字节数据 + ADD8校验 + 03) "
        )
        frm_write.pack(fill='x', padx=8, pady=6)

        ttk.Label(frm_write, text="写入方式:").grid(row=0, column=0, padx=4, pady=6, sticky='e')
        self.cmb_wmode = ttk.Combobox(
            frm_write, width=14, state='readonly',
            values=["十进制编号", "HEX数据(4字节)"]
        )
        self.cmb_wmode.set("十进制编号")
        self.cmb_wmode.grid(row=0, column=1, padx=4)
        self.cmb_wmode.bind("<<ComboboxSelected>>", self._wmode_hint)

        ttk.Label(frm_write, text="数据:").grid(row=0, column=2, padx=4, sticky='e')
        self.ent_wdata = ttk.Entry(frm_write, width=22)
        self.ent_wdata.grid(row=0, column=3, padx=4)
        self.ent_wdata.insert(0, "00 00 00 5D")

        self.btn_write = ttk.Button(frm_write, text="写入", command=self.write_tag)
        self.btn_write.grid(row=0, column=4, padx=8)

        self.var_auto_read_after_write = tk.BooleanVar(value=True)
        self.chk_auto_read = ttk.Checkbutton(
            frm_write, text="写入成功后自动读取",
            variable=self.var_auto_read_after_write
        )
        self.chk_auto_read.grid(row=0, column=5, padx=8)

        self.var_auto_inc = tk.BooleanVar(value=True)
        self.chk_auto_inc = ttk.Checkbutton(
            frm_write, text="写后数据自动+1",
            variable=self.var_auto_inc
        )
        self.chk_auto_inc.grid(row=1, column=5, padx=8, sticky='w')

        self.lbl_wframe = ttk.Label(frm_write, text="待发送帧: -", foreground="#888888")
        self.lbl_wframe.grid(row=1, column=0, columnspan=6, padx=6, pady=4, sticky='w')

        self.ent_wdata.bind("<KeyRelease>", lambda e: self._preview_frame())
        self.cmb_wmode.bind("<<ComboboxSelected>>", lambda e: self._preview_frame())
        self._wmode_hint()

        # ===== 日志区：写入 / 读取分开 =====
        frm_logs = ttk.Frame(self)
        frm_logs.pack(fill='both', expand=True, padx=8, pady=6)

        # ---------- 写入日志 ----------
        frm_write_log = ttk.LabelFrame(frm_logs, text=" 写入日志 (TX) ")
        frm_write_log.pack(side='left', fill='both', expand=True, padx=(0, 4))

        self.txt_write_log = scrolledtext.ScrolledText(
            frm_write_log, height=16, font=("Consolas", 10)
        )
        self.txt_write_log.pack(fill='both', expand=True, padx=4, pady=4)
        self.txt_write_log.tag_config('tx', foreground='#007700')
        self.txt_write_log.tag_config('ok', foreground='#009900',
                                      font=("Consolas", 10, "bold"))
        self.txt_write_log.tag_config('err', foreground='#CC0000',
                                      font=("Consolas", 10, "bold"))
        self.txt_write_log.tag_config('info', foreground='#666666')

        ttk.Button(
            frm_write_log, text="清空写日志",
            command=self.clear_write_log
        ).pack(anchor='e', padx=4, pady=2)

        # ---------- 读取日志 ----------
        frm_read_log = ttk.LabelFrame(frm_logs, text=" 读取日志 (RX) ")
        frm_read_log.pack(side='left', fill='both', expand=True, padx=(4, 0))

        self.txt_read_log = scrolledtext.ScrolledText(
            frm_read_log, height=16, font=("Consolas", 10)
        )
        self.txt_read_log.pack(fill='both', expand=True, padx=4, pady=4)
        self.txt_read_log.tag_config('rx', foreground='#0000AA')
        self.txt_read_log.tag_config('ok', foreground='#009900',
                                     font=("Consolas", 10, "bold"))
        self.txt_read_log.tag_config('err', foreground='#CC0000',
                                     font=("Consolas", 10, "bold"))
        self.txt_read_log.tag_config('info', foreground='#666666')

        ttk.Button(
            frm_read_log, text="清空读日志",
            command=self.clear_read_log
        ).pack(anchor='e', padx=4, pady=2)

    # ---------- 串口列表 / 手动输入 ----------
    def _toggle_manual_port(self):
        """切换串口下拉框是否可手动编辑"""
        if self.var_manual_port.get():
            self.cmb_port.config(state='normal')
            if not self.cmb_port.get():
                # 给个默认值，方便直接改
                self.cmb_port.set("/dev/ttyCH341USB0")
            self.cmb_port.focus_set()
        else:
            self.cmb_port.config(state='readonly')
            self.refresh_ports()

    def refresh_ports(self):
        if list_ports is None:
            self.cmb_port['values'] = ["pyserial 未安装"]
            return

        ports = [p.device for p in list_ports.comports()]

        # 补充 pyserial 枚举不到的节点（如 CH341 官方驱动的 ttyCH341USB*）
        for dev in glob.glob('/dev/ttyCH341USB*') + glob.glob('/dev/ttyUSB*'):
            if dev not in ports:
                ports.append(dev)

        self.cmb_port['values'] = ports

        # 仅在非手动模式下自动选一个合理的默认值
        if ports and not self.var_manual_port.get():
            preferred = [p for p in ports if 'ttyUSB' in p or 'ttyCH341' in p]
            self.cmb_port.set(preferred[0] if preferred else ports[0])

    # ---------- 写入数据辅助 ----------
    def _increment_wdata(self):
        """写入成功后数据自动 +1 (按当前模式回显)"""
        d = self._get_wdata()
        if d is None:
            return
        n = (int.from_bytes(d, 'big') + 1) & 0xFFFFFFFF
        self.ent_wdata.delete(0, 'end')
        if self.cmb_wmode.get() == "HEX数据(4字节)":
            self.ent_wdata.insert(0, n.to_bytes(4, 'big').hex(' ').upper())
        else:
            self.ent_wdata.insert(0, str(n))
        self._preview_frame()

    def _wmode_hint(self, event=None):
        """切换写入模式时，填入合理的默认值"""
        if self.cmb_wmode.get() == "十进制编号":
            self.ent_wdata.delete(0, 'end')
            self.ent_wdata.insert(0, "93")
        else:
            self.ent_wdata.delete(0, 'end')
            self.ent_wdata.insert(0, "11 11 11 11")
        self._preview_frame()

    # ---------- 日志 ----------
    def _timestamp(self):
        return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

    def write_log(self, msg, tag='info'):
        """写入日志：只记录 TX / 写入相关信息。"""
        def append():
            ts = self._timestamp()
            self.txt_write_log.insert('end', f"[{ts}] {msg}\n", tag)
            self.txt_write_log.see('end')

        if threading.current_thread() is threading.main_thread():
            append()
        else:
            self.after(0, append)

    def read_log(self, msg, tag='info'):
        """读取日志：只记录 RX / 读取相关信息。"""
        def append():
            ts = self._timestamp()
            self.txt_read_log.insert('end', f"[{ts}] {msg}\n", tag)
            self.txt_read_log.see('end')

        if threading.current_thread() is threading.main_thread():
            append()
        else:
            self.after(0, append)

    def clear_write_log(self):
        self.txt_write_log.delete('1.0', 'end')

    def clear_read_log(self):
        self.txt_read_log.delete('1.0', 'end')

    # ---------- 数据解析 ----------
    def _get_wdata(self):
        """按当前模式解析 4 字节数据, 失败返回 None"""
        mode = self.cmb_wmode.get()
        text = self.ent_wdata.get().strip()
        try:
            if mode == "十进制编号":
                n = int(text, 10)
                if not (0 <= n <= 0xFFFFFFFF):
                    raise ValueError
                return n.to_bytes(4, 'big')
            else:
                parts = text.replace(',', ' ').split()
                # 兼容 SSCOM 风格连写: 11111111 -> 11 11 11 11
                if len(parts) == 1 and len(parts[0]) == 8:
                    parts = [parts[0][i:i+2] for i in range(0, 8, 2)]
                b = bytes(int(x, 16) for x in parts)
                if len(b) != 4:
                    raise ValueError
                return b
        except ValueError:
            return None

    def _preview_frame(self):
        d = self._get_wdata()
        if d is None:
            self.lbl_wframe.config(text="待发送帧: 数据格式错误", foreground="#CC0000")
            return
        frame = build_write_frame(d)
        self.lbl_wframe.config(
            text=f"待发送帧: {frame.hex(' ').upper()}   (校验和 {add8(WRITE_HEAD + d):02X})",
            foreground="#006600"
        )

    # ---------- 串口开关 ----------
    def toggle_serial(self):
        if self.ser and self.ser.is_open:
            # 关闭
            self.auto_read_on = False
            self.running = False
            time.sleep(0.2)
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
            self.btn_open.config(text="打开串口")
            self.write_log("串口已关闭")
            return

        # 打开
        if serial is None:
            messagebox.showerror("错误", "未安装 pyserial，请先执行:  pip install pyserial")
            return

        port = self.cmb_port.get().strip()
        if not port:
            messagebox.showwarning("提示", "请选择或输入串口")
            return

        # 设备存在性检查（仅对 /dev/ 下的路径做，避免 Windows COMx 误判）
        if port.startswith('/dev/') and not os.path.exists(port):
            messagebox.showerror("打开失败", f"设备节点不存在:\n{port}")
            return

        try:
            self.ser = serial.Serial(
                port, int(self.cmb_baud.get()),
                bytesize=8, parity='N', stopbits=1, timeout=0.05
            )
        except Exception as e:
            messagebox.showerror("打开失败", str(e))
            return

        self.running = True
        self.rx_buf.clear()
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
        self.btn_open.config(text="关闭串口")
        self.write_log(f"串口 {port} 已打开  {self.cmb_baud.get()},8,N,1", 'ok')

    # ---------- 发送 ----------
    def send(self, frame: bytes, desc=""):
        if not (self.ser and self.ser.is_open):
            self.write_log("串口未打开", 'err')
            return False
        try:
            with self.ser_lock:
                self.ser.write(frame)
        except Exception as e:
            self.write_log(f"发送失败: {e}", 'err')
            return False
        self.write_log(f"TX  {frame.hex(' ').upper()}  {desc}", 'tx')
        return True

    def read_once(self):
        if not (self.ser and self.ser.is_open):
            messagebox.showwarning("提示", "请先打开串口")
            return
        self.send(READ_CMD, "读取标签")

    def toggle_auto_read(self):
        if self.auto_read_on:
            self.auto_read_on = False
            self.btn_auto.config(text="开始自动读取")
            self.write_log("自动读取已停止")
        else:
            if not (self.ser and self.ser.is_open):
                messagebox.showwarning("提示", "请先打开串口")
                return
            self.auto_read_on = True
            self.btn_auto.config(text="停止自动读取")
            self.write_log("自动读取已开始", 'ok')
            t = threading.Thread(target=self._auto_read_loop, daemon=True)
            t.start()

    def _auto_read_loop(self):
        while self.auto_read_on and self.ser and self.ser.is_open:
            try:
                with self.ser_lock:
                    self.ser.write(READ_CMD)
                self.write_log(
                    f"TX  {READ_CMD.hex(' ').upper()}  自动读取",
                    'tx'
                )
            except Exception:
                break
            try:
                interval = max(50, int(self.spin_interval.get()))
            except ValueError:
                interval = 500
            time.sleep(interval / 1000.0)

    def write_tag(self):
        d = self._get_wdata()
        if d is None:
            messagebox.showwarning(
                "提示",
                "数据格式错误：\n十进制编号: 0~4294967295\nHEX: 4 个字节, 如 11 11 11 11"
            )
            return
        frame = build_write_frame(d)
        # 注: 与 SSCOM 不同, 本程序自行计算 ADD8 校验后直接发送完整帧
        self.send(frame, f"写入标签 (校验和 {add8(WRITE_HEAD + d):02X})")

    # ---------- 接收解析 ----------
    def _rx_loop(self):
        while self.running and self.ser and self.ser.is_open:
            try:
                chunk = self.ser.read(64)
            except Exception as e:
                self.read_log(f"串口接收异常: {e}", 'err')
                break
            if chunk:
                self.rx_buf.extend(chunk)
                self._parse_buffer()
        self.running = False

    def _parse_buffer(self):
        """从缓冲区提取完整帧并处理 (所有帧均以 03 结尾)"""
        while True:
            try:
                idx = self.rx_buf.index(TAIL)
            except ValueError:
                if len(self.rx_buf) > 512:      # 防止脏数据撑爆缓冲
                    self.rx_buf.clear()
                return
            frame = bytes(self.rx_buf[:idx + 1])
            del self.rx_buf[:idx + 1]
            self.after(0, self._handle_frame, frame)

    def _handle_frame(self, frame: bytes):
        hx = frame.hex(' ').upper()

        # --- 写入应答: 30 30 03 成功 / 34 34 03 失败 ---
        if frame == bytes([0x30, 0x30, 0x03]):
            self.read_log(f"RX  {hx}  >> 写入成功", 'ok')
            # 写成功后，如果开启自动+1，则数据+1
            if self.var_auto_inc.get():
                self._increment_wdata()
            if self.var_auto_read_after_write.get():
                # 延迟 200ms 自动读取一次，确认写入结果
                self.after(200, lambda: self.send(READ_CMD, "写入成功,自动读取"))
            return

        if frame == bytes([0x34, 0x34, 0x03]):
            self.read_log(f"RX  {hx}  >> 写入失败", 'err')
            return

        # --- 标签离开 ---
        if frame == bytes([0x35, 0x35, 0x03]):
            self.read_log(f"RX  {hx}  >> 标签离开", 'err')
            self.lbl_last.config(text="当前标签: 无")
            return

        # --- 应答标签信息: 30 + 4字节数据 + 校验 + 03 (7 字节) ---
        if len(frame) == 7 and frame[0] == 0x30 and frame[6] == 0x03:
            data4 = frame[1:5]
            cs = add8(frame[:5])
            if cs == frame[5]:
                info = parse_tag_info(data4)
                self.read_log(f"RX  {hx}  >> {info}", 'rx')
                self.lbl_last.config(text=f"当前标签: {info}")
            else:
                self.read_log(
                    f"RX  {hx}  >> 校验错误 (期望 {cs:02X}, 收到 {frame[5]:02X})",
                    'err'
                )
            return

        # --- 其它未知帧 ---
        self.read_log(f"RX  {hx}  >> 未知帧", 'err')

    def on_close(self):
        self.auto_read_on = False
        self.running = False
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.destroy()


if __name__ == "__main__":
    app = TagHostApp()
    app.protocol("WM_DELETE_WINDOW", app.on_close)
    app.mainloop()
