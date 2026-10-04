"""独立的模拟抓握调试窗。实机SDK后端由经验证的主站工作进程调用。"""
import json
import os
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk, messagebox
from .profiles import CHANNELS, GraspLibrary
from .control import HandController, SimHandBackend, TriggerSample

ROOT = Path(__file__).resolve().parents[3]


class GraspWindow:
    def __init__(self, root, library_path=None):
        self.root = root
        root.title('DH116 抓握动作调试 · 模拟模式')
        root.geometry('1160x870')
        root.minsize(1060, 830)
        self.library = GraspLibrary(library_path or ROOT/'config/dh116_grasps.json')
        self.backend = SimHandBackend()
        self.controller = HandController(self.library, self.backend)
        self.sequence = 0
        self.trigger = tk.DoubleVar(value=0)
        self.grip = tk.BooleanVar(value=False)
        self.status = tk.StringVar(value='等待输入')
        self.selected = tk.StringVar(value='圆柱包握（瓶身）')
        self.name = tk.StringVar(value='我的抓握')
        self.description = tk.StringVar()
        self.editors = [tk.DoubleVar(value=round(v*100, 1)) for v in self.library.profiles['cylinder']['closed']]
        self.queue = []
        self.queue_active = False
        self.queue_phase = None
        self.queue_start = 0.
        self.queue_hold = None
        self.closed = False
        self.log_path = ROOT/'logs'/f'dh116_grasps_{time.strftime("%Y%m%d_%H%M%S")}_{os.getpid()}.jsonl'
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_file = self.log_path.open('w', encoding='utf-8', buffering=1)
        self.previous_state = None
        self._build()
        self._describe()
        self._note('启动模拟；预置动作是可调草案，未连接实机。')
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.bind('<Escape>', lambda _: self.stop())
        root.after(30, self._tick)

    def _build(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TFrame', background='#f3f5f8')
        style.configure('TLabel', background='#f3f5f8', font=('Microsoft YaHei UI', 10))
        style.configure('TButton', font=('Microsoft YaHei UI', 10), padding=7)
        style.configure('Title.TLabel', font=('Microsoft YaHei UI', 19, 'bold'))
        page = ttk.Frame(self.root, padding=18)
        page.pack(fill='both', expand=True)
        ttk.Label(page, text='DH116  抓握动作调试', style='Title.TLabel').pack(anchor='w')
        ttk.Label(page, text='模拟模式  ·  六通道动作草案  ·  Trigger 连续开合  ·  无实机连接',
                  foreground='#216c59').pack(anchor='w', pady=(4, 12))
        body = ttk.Frame(page)
        body.pack(fill='both', expand=True)
        body.columnconfigure(0, weight=0, minsize=265)
        body.columnconfigure(1, weight=1, minsize=360)
        body.columnconfigure(2, weight=0, minsize=355)
        body.rowconfigure(0, weight=1)
        left = ttk.Frame(body, padding=(0, 0, 14, 0))
        left.grid(row=0, column=0, sticky='nsew')
        ttk.Label(left, text='选择抓握动作', font=('Microsoft YaHei UI', 12, 'bold')).pack(anchor='w')
        self.action_buttons = {}
        self.button_frame = ttk.Frame(left)
        self.button_frame.pack(fill='x', pady=8)
        self._refresh_buttons()
        self.description_label = ttk.Label(left, textvariable=self.description, wraplength=245, foreground='#637087')
        self.description_label.pack(anchor='w', pady=8)
        ttk.Label(left, text='模拟动作队列', font=('Microsoft YaHei UI', 11, 'bold')).pack(anchor='w', pady=(16, 5))
        self.queue_list = tk.Listbox(left, height=4, font=('Microsoft YaHei UI', 10), relief='flat')
        self.queue_list.pack(fill='x')
        controls = ttk.Frame(left)
        controls.pack(fill='x', pady=5)
        ttk.Button(controls, text='加入', command=self.add_queue).pack(side='left')
        ttk.Button(controls, text='清空', command=self.clear_queue).pack(side='left', padx=3)
        self.run_button = ttk.Button(left, text='运行队列（模拟）', command=self.run_queue)
        self.run_button.pack(fill='x')
        center = ttk.Frame(body, padding=(0, 0, 14, 0))
        center.grid(row=0, column=1, sticky='nsew')
        ttk.Label(center, textvariable=self.selected, font=('Microsoft YaHei UI', 12, 'bold')).pack(anchor='w')
        self.canvas = tk.Canvas(center, width=350, height=240, background='white', highlightthickness=0)
        self.canvas.pack(fill='x', pady=10)
        ttk.Label(center, text='通道示意，非碰撞仿真或真实手指几何', foreground='#6f7784').pack(anchor='w')
        ttk.Label(center, text='Trigger 开合比例', font=('Microsoft YaHei UI', 11, 'bold')).pack(anchor='w', pady=(18, 4))
        self.trigger_scale = ttk.Scale(center, from_=0, to=1, variable=self.trigger)
        self.trigger_scale.pack(fill='x')
        self.trigger_label = ttk.Label(center, text='0%')
        self.trigger_label.pack(anchor='e')
        self.grip_button = ttk.Checkbutton(center, text='Grip 使能（勾选后可开合）', variable=self.grip)
        self.grip_button.pack(anchor='w', pady=6)
        self.open_button = ttk.Button(center, text='Trigger 归零（使能时逐渐张开）', command=lambda: self.trigger.set(0))
        self.open_button.pack(fill='x', pady=3)
        self.stop_button = ttk.Button(center, text='停止并锁定  [Esc]', command=self.stop)
        self.stop_button.pack(fill='x', pady=3)
        self.reset_button = ttk.Button(center, text='恢复（先松开 Trigger / Grip）', command=self.reset)
        self.reset_button.pack(fill='x', pady=3)
        right = ttk.Frame(body)
        right.grid(row=0, column=2, sticky='nsew')
        ttk.Label(right, text='编辑闭合端点 / 当前通道', font=('Microsoft YaHei UI', 12, 'bold')).pack(anchor='w')
        self.feedback_labels = []
        for index, label in enumerate(CHANNELS):
            row = ttk.Frame(right)
            row.pack(fill='x', pady=(10, 2))
            ttk.Label(row, text=f'{index+1}  {label}').pack(side='left')
            value = ttk.Label(row, text='当前 0.0%', foreground='#277263')
            value.pack(side='right')
            self.feedback_labels.append(value)
            slider = ttk.Frame(right)
            slider.pack(fill='x')
            ttk.Scale(slider, from_=0, to=100, variable=self.editors[index]).pack(side='left', fill='x', expand=True)
            ttk.Spinbox(slider, from_=0, to=100, textvariable=self.editors[index], width=6, increment=1, format='%.1f').pack(side='right')
        self.preview_button = ttk.Button(right, text='预览编辑值（模拟限速执行）', command=self.preview)
        self.preview_button.pack(fill='x', pady=(15, 5))
        ttk.Entry(right, textvariable=self.name).pack(fill='x', pady=3)
        self.save_button = ttk.Button(right, text='保存为自定义动作', command=self.save)
        self.save_button.pack(fill='x')
        ttk.Label(right, text='编辑和切换前松开 Trigger / Grip。\n端点百分比不代表角度、压力或抓握力。',
                  foreground='#6f7784').pack(anchor='w', pady=8)
        ttk.Separator(page).pack(fill='x', pady=(10, 8))
        ttk.Label(page, textvariable=self.status, foreground='#184e70').pack(anchor='w')
        self.messages = tk.Text(page, height=4, font=('Microsoft YaHei UI', 9), background='white', relief='flat', state='disabled')
        self.messages.pack(fill='x', pady=(7, 0))

    def _refresh_buttons(self):
        for widget in self.button_frame.winfo_children():
            widget.destroy()
        for key, profile in self.library.profiles.items():
            button = ttk.Button(self.button_frame, text=profile['name'], command=lambda key=key: self.choose(key))
            button.pack(fill='x', pady=2)
            self.action_buttons[key] = button

    def _note(self, text):
        self.messages.configure(state='normal')
        self.messages.insert('end', time.strftime('%H:%M:%S') + '  ' + text + '\n')
        self.messages.see('end')
        self.messages.configure(state='disabled')

    def _describe(self):
        p = self.library.profiles[self.controller.profile]
        self.selected.set(p['name'])
        self.description.set(p['description'])

    def choose(self, key):
        try:
            if self.queue_active or self.trigger.get() > .05 or self.grip.get():
                raise RuntimeError('请停止队列并松开Trigger和Grip再切换动作')
            self.controller.select(key)
            self._describe()
            for variable, value in zip(self.editors, self.library.profiles[key]['closed']):
                variable.set(round(value*100, 1))
            self._note('选择：' + self.selected.get())
        except Exception as exc:
            self._note(str(exc))

    def preview(self):
        try:
            if self.queue_active or self.grip.get() or self.trigger.get() > .05:
                raise RuntimeError('编辑预览前先松开Trigger和Grip')
            self.controller.set_manual([v.get()/100 for v in self.editors])
            self.grip.set(True)
            self._note('预览六通道编辑值，完成后松开Grip；停止可按Esc。')
        except Exception as exc:
            self._note(str(exc))

    def save(self):
        try:
            if self.queue_active or self.grip.get() or self.trigger.get() > .05:
                raise RuntimeError('保存前先松开Trigger和Grip')
            self.library.save_custom(self.name.get(), [v.get()/100 for v in self.editors])
            self._refresh_buttons()
            self.choose('custom')
            self._note('已保存至动作库；自定义动作仍为模拟草案。')
        except Exception as exc:
            self._note(str(exc))

    def add_queue(self):
        if self.queue_active:
            return
        self.queue.append(self.controller.profile)
        self.queue_list.insert('end', self.selected.get())

    def clear_queue(self):
        if not self.queue_active:
            self.queue.clear()
            self.queue_list.delete(0, 'end')

    def run_queue(self):
        if self.queue_active or not self.queue:
            self._note('先选择动作并加入队列。')
            return
        if self.trigger.get() > .05 or self.grip.get() or self.controller.fault:
            self._note('运行前请松开Trigger和Grip，并恢复停止锁定。')
            return
        self.queue_active = True
        self.queue_index = 0
        self.queue_phase = 'select'
        self.queue_start = time.monotonic()
        self._note('开始模拟队列：逐个闭合、保持、张开。')

    def _queue_tick(self, now):
        if not self.queue_active:
            return
        if self.controller.fault:
            self.queue_active = False
            self.grip.set(False)
            return
        if now-self.queue_start > 12:
            self.stop('模拟队列阶段超时')
            return
        if self.queue_phase == 'select':
            self.trigger.set(0)
            self.grip.set(False)
            self.controller.select(self.queue[self.queue_index])
            self._describe()
            self.queue_phase = 'open'
            return  # 给状态机一帧解除Grip，再开始下一动作。
        self.grip.set(True)
        value = 1 if self.queue_phase in ('close', 'hold') else 0
        self.trigger.set(value)
        target = self.library.target(self.controller.profile, value)
        reached = self.controller.q is not None and max(abs(a-b) for a, b in zip(target, self.controller.q)) < .005
        if not reached:
            return
        if self.queue_phase == 'open':
            self.queue_phase = 'close'
        elif self.queue_phase == 'close':
            self.queue_phase, self.queue_hold = 'hold', now
        elif self.queue_phase == 'hold' and now-self.queue_hold > .7:
            self.queue_phase = 'release'
        elif self.queue_phase == 'release':
            self.grip.set(False)
            self.queue_index += 1
            if self.queue_index == len(self.queue):
                self.queue_active = False
                self._note('模拟队列完成。')
            else:
                self.queue_phase = 'select'
        self.queue_start = now

    def stop(self, reason='操作者停止'):
        self.queue_active = False
        self.grip.set(False)
        self.controller.stop(reason)
        self._note('停止并锁定：保持当前位置，恢复须松开Trigger和Grip。')

    def reset(self):
        try:
            if self.grip.get() or self.trigger.get() > .05:
                raise RuntimeError('请先松开Trigger和Grip')
            now = time.monotonic()
            self.sequence += 1
            self.controller.submit(TriggerSample(self.trigger.get(), now, self.sequence, False))
            self.controller.reset(now)
            self._note('恢复完成。')
        except Exception as exc:
            self._note(str(exc))

    def _draw(self):
        c = self.canvas
        c.delete('all')
        q = self.controller.q or (0.,)*6
        c.create_rectangle(90, 135, 280, 210, fill='#e4edf3', outline='#b4c9d5', width=2)
        # 弯曲量以长度收缩表示，仅用于查看通道是否按预期变化。
        for index, x in enumerate((115, 160, 205, 250)):
            length = 100*(1-.7*q[index+2])
            c.create_rectangle(x-13, 135-length, x+13, 135, fill='#458b84', outline='', width=0)
            c.create_text(x, 225, text=('食', '中', '环', '小')[index], fill='#4f6170')
        x = 40+q[0]*40
        c.create_line(95, 175, x, 140+q[1]*40, width=24, fill='#dd996a', capstyle='round')
        c.create_text(52, 110, text='拇指', fill='#896347')
        c.create_text(185, 172, text='DH116 · SIM', fill='#668194', font=('Segoe UI', 12, 'bold'))

    def _tick(self):
        if self.closed:
            return
        now = time.monotonic()
        try:
            self._queue_tick(now)
            self.sequence += 1
            self.controller.submit(TriggerSample(self.trigger.get(), now, self.sequence, bool(self.grip.get())))
            self.controller.tick(now)
            if self.controller.fault and self.queue_active:
                self.queue_active = False
                self.grip.set(False)
            state = self.controller.state
            self.status.set('模拟  |  ' + state + '  |  实机抓握与接触力尚未验收')
            if state != self.previous_state:
                self._note(state)
                self.previous_state = state
            self.trigger_label.configure(text=f'{self.trigger.get()*100:.0f}%')
            for label, value in zip(self.feedback_labels, self.controller.q or (0.,)*6):
                label.configure(text=f'当前 {value*100:.1f}%')
            self._draw()
            self.log_file.write(json.dumps(dict(stamp=now, mode='SIM', profile=self.controller.profile,
                trigger=self.trigger.get(), grip=self.grip.get(), state=state, position=self.controller.q,
                fault=self.controller.fault), ensure_ascii=False) + '\n')
        except Exception as exc:
            self.stop(str(exc))
        self.root.after(30, self._tick)

    def close(self):
        if self.closed:
            return
        self.controller.stop('关闭窗口')
        self.closed = True
        self.log_file.close()
        self.root.destroy()


def main():
    root = tk.Tk()
    GraspWindow(root)
    root.mainloop()
