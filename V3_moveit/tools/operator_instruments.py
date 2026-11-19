"""Reusable display widgets; no robot I/O."""
import json,math,mmap,os,signal,struct,subprocess,time
from collections import deque
from pathlib import Path
from PyQt5 import QtCore,QtGui,QtWidgets as W
from operator_settings import ROOT,atomic
SIGNAL_COLORS=['#70e5c1','#71b7ff','#f5cf69','#be9bff','#ff91a7','#71dbe7']
class Scope(W.QWidget):
    def __init__(self):
        super().__init__();self.samples=deque(maxlen=4000);self.selected=list(range(6));self.fresh=False;self.seconds=10;self.max_gap=.25;self.unit='°';self.paused=False;self.frozen_at=0.;self.setMinimumHeight(155)
    def push(self,values,stamp=None):
        if self.paused:return
        if len(values)==6 and all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):self.samples.append((stamp or time.monotonic(),list(values)));self.fresh=True;self.update()
    def freeze(self,yes):self.paused=yes;self.frozen_at=time.monotonic();self.update()
    def paintEvent(self,event):
        p=QtGui.QPainter(self);p.setRenderHint(QtGui.QPainter.Antialiasing);p.fillRect(self.rect(),QtGui.QColor('#0c131e'))
        area=QtCore.QRectF(55,16,max(1,self.width()-74),max(1,self.height()-47));now=self.frozen_at if self.paused else time.monotonic()
        points=[(t,q) for t,q in self.samples if now-self.seconds<=t<=now];values=[q[i] for _,q in points for i in self.selected]
        low=math.floor((min(values)-2)/5)*5 if values else -90;high=math.ceil((max(values)+2)/5)*5 if values else 90
        for i in range(21):
            p.setPen(QtGui.QColor('#28384a' if i%5==0 else '#172331'));x=area.left()+area.width()*i/20;p.drawLine(QtCore.QPointF(x,area.top()),QtCore.QPointF(x,area.bottom()))
        for i in range(9):
            y=area.top()+area.height()*i/8;p.setPen(QtGui.QColor('#28384a' if i%2==0 else '#172331'));p.drawLine(QtCore.QPointF(area.left(),y),QtCore.QPointF(area.right(),y))
            if i%2==0:
                p.setPen(QtGui.QColor('#8396ae'));p.drawText(QtCore.QRectF(0,y-10,48,20),QtCore.Qt.AlignRight|QtCore.Qt.AlignVCenter,f'{high-(high-low)*i/8:.0f}')
        p.setPen(QtGui.QColor('#8396ae'));p.drawText(8,14,self.unit)
        for i in range(5):p.drawText(QtCore.QRectF(area.left()+area.width()*i/4-24,area.bottom()+6,48,20),QtCore.Qt.AlignCenter,f'{-self.seconds+self.seconds*i/4:g}s')
        for idx in self.selected:
            path=QtGui.QPainterPath();prev=None
            for t,q in points:
                xy=QtCore.QPointF(area.right()-(now-t)/self.seconds*area.width(),area.bottom()-(q[idx]-low)/(high-low)*area.height())
                if prev is None or t-prev>self.max_gap:path.moveTo(xy)
                else:path.lineTo(xy)
                prev=t
            p.setPen(QtGui.QPen(QtGui.QColor(SIGNAL_COLORS[idx] if self.fresh else '#69778a'),1.4));p.drawPath(path)
        if not self.selected or not points:
            p.setPen(QtGui.QColor('#8b9cb3'));p.drawText(area,QtCore.Qt.AlignCenter,'选择至少一个信号' if not self.selected else '等待实时反馈')
        if self.paused:p.setPen(QtGui.QColor('#f5cf69'));p.drawText(QtCore.QRectF(area.right()-130,20,120,20),QtCore.Qt.AlignRight,'PAUSED / 已暂停')
def fitted_rect(source, bounds):
    """One scale for both axes, including while a resized render is pending."""
    if source.width()<=0 or source.height()<=0:return QtCore.QRectF()
    scale=min(bounds.width()/source.width(),bounds.height()/source.height())
    w,h=source.width()*scale,source.height()*scale
    return QtCore.QRectF(bounds.center().x()-w/2,bounds.center().y()-h/2,w,h)

def render_size(width,height,dpr=1.):
    scale=min(dpr,1600/max(1,width),900/max(1,height))
    return max(1,round(width*scale)),max(1,round(height*scale))

class CameraTile(W.QWidget):
    activated=QtCore.pyqtSignal()
    frame_updated=QtCore.pyqtSignal()
    def __init__(self,title,subtitle):
        super().__init__();self.title=title;self.subtitle=subtitle;self.pixmap=None;self.detail='未连接';self.ok=False;self.indicator_inset=14
        self.setStyleSheet('background:transparent;');self.setAttribute(QtCore.Qt.WA_TranslucentBackground);self.setCursor(QtCore.Qt.PointingHandCursor);self.setFocusPolicy(QtCore.Qt.StrongFocus);self.setAccessibleName(title+'，按下查看大图')
        shadow=W.QGraphicsDropShadowEffect(self);shadow.setBlurRadius(20);shadow.setOffset(0,4);shadow.setColor(QtGui.QColor(0,0,0,80));self.setGraphicsEffect(shadow)
    def update_frame(self,path,ok,detail):
        self.ok=bool(ok);self.detail=detail;self.setToolTip(self.title+' · '+detail+'\n点击查看 · Esc 返回')
        if ok:
            pix=QtGui.QPixmap(str(path))
            if not pix.isNull():self.pixmap=pix
            else:self.pixmap=None;self.ok=False;self.detail='图像文件不可读'
        else:self.pixmap=None
        self.update();self.frame_updated.emit()
    def mousePressEvent(self,event):event.accept()
    def mouseMoveEvent(self,event):event.accept()
    def wheelEvent(self,event):event.accept()
    def mouseReleaseEvent(self,event):
        if event.button()==QtCore.Qt.LeftButton and self.rect().contains(event.pos()):self.activated.emit()
    def keyPressEvent(self,event):
        if event.key() in (QtCore.Qt.Key_Return,QtCore.Qt.Key_Space):self.activated.emit()
        else:super().keyPressEvent(event)
    def paintEvent(self,event):
        p=QtGui.QPainter(self);p.setRenderHints(QtGui.QPainter.Antialiasing|QtGui.QPainter.SmoothPixmapTransform)
        area=QtCore.QRectF(self.rect()).adjusted(.5,.5,-.5,-.5);path=QtGui.QPainterPath();path.addRoundedRect(area,12,12);p.setClipPath(path)
        p.fillRect(self.rect(),QtGui.QColor('#17191e'))
        if self.pixmap is not None:p.drawPixmap(fitted_rect(self.pixmap.size(),area),self.pixmap,QtCore.QRectF(self.pixmap.rect()))
        else:
            p.setPen(QtGui.QColor('#aeb2bc'));font=p.font();font.setPixelSize(11);p.setFont(font)
            p.drawText(area.adjusted(12,33,-12,-12),QtCore.Qt.AlignCenter|QtCore.Qt.TextWordWrap,self.detail)
        # Labels sit over the picture; no separate header, footer or outer panel.
        shade=QtGui.QLinearGradient(0,0,0,48);shade.setColorAt(0,QtGui.QColor(0,0,0,155));shade.setColorAt(1,QtGui.QColor(0,0,0,0));p.fillRect(QtCore.QRectF(0,0,self.width(),48),shade)
        font=p.font();font.setPixelSize(12);font.setWeight(QtGui.QFont.Medium);p.setFont(font);p.setPen(QtGui.QColor('#ffffff'));p.drawText(12,23,self.title)
        p.setBrush(QtGui.QColor('#66e2a5' if self.ok else '#b0b4be'));p.setPen(QtCore.Qt.NoPen);p.drawEllipse(QtCore.QPointF(self.width()-self.indicator_inset,18),3,3)
        if self.ok and self.subtitle:
            font.setPixelSize(10);font.setWeight(QtGui.QFont.Normal);p.setFont(font);text_width=p.fontMetrics().horizontalAdvance(self.subtitle)
            pill=QtCore.QRectF(9,self.height()-28,min(text_width+16,self.width()-18),20);p.setBrush(QtGui.QColor(15,17,22,165));p.drawRoundedRect(pill,9,9);p.setPen(QtGui.QColor('#f2f3f7'));p.drawText(pill,QtCore.Qt.AlignCenter,self.subtitle)
        p.setClipping(False);p.setBrush(QtCore.Qt.NoBrush);p.setPen(QtGui.QPen(QtGui.QColor('#0a84ff') if self.hasFocus() else QtGui.QColor(255,255,255,45),2 if self.hasFocus() else 1));p.drawRoundedRect(area,12,12)

class CameraOverlay(W.QWidget):
    """Transparent left rail. Only its three image widgets occupy visual space."""
    def __init__(self,parent):
        super().__init__(parent);self.setAttribute(QtCore.Qt.WA_TranslucentBackground);self.setStyleSheet('background:transparent;');self.tiles=[]
    def arrange(self):
        for i,tile in enumerate(self.tiles):tile.setGeometry(0,i*(self.width()*9//16+12),self.width(),self.width()*9//16)

class CameraFocus(W.QWidget):
    def __init__(self,parent):
        super().__init__(parent);self.source=None;self.setAttribute(QtCore.Qt.WA_TranslucentBackground);self.setStyleSheet('background:transparent;');self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.preview=CameraTile('','');self.preview.indicator_inset=64;self.preview.setParent(self);self.preview.setGraphicsEffect(None);self.preview.setCursor(QtCore.Qt.ArrowCursor);self.preview.setFocusPolicy(QtCore.Qt.NoFocus)
        self.close_button=W.QToolButton(self);self.close_button.setText('×');self.close_button.setAccessibleName('关闭相机大图');self.close_button.setToolTip('关闭 · Esc');self.close_button.setStyleSheet('QToolButton{background:rgba(25,27,33,205);color:white;border:1px solid rgba(255,255,255,70);border-radius:17px;font-size:22px;} QToolButton:hover{background:#41444c;}');self.close_button.setFixedSize(34,34);self.close_button.clicked.connect(self.dismiss);self.hide()
    def sync(self):
        if self.source is None:return
        for key in ('title','subtitle','pixmap','detail','ok'):setattr(self.preview,key,getattr(self.source,key))
        self.preview.update()
    def reveal(self,source):
        self.source=source;self.sync();self.setGeometry(self.parentWidget().rect());self.show();self.raise_();self.setFocus()
    def resizeEvent(self,event):
        target=fitted_rect(QtCore.QSize(16,9),QtCore.QRectF(self.rect()).adjusted(36,36,-36,-36)).toRect();self.preview.setGeometry(target);self.close_button.move(target.right()-44,target.top()+10);self.close_button.raise_()
    def dismiss(self):
        self.hide()
        if self.source:self.source.setFocus()
    def keyPressEvent(self,event):
        if event.key()==QtCore.Qt.Key_Escape:self.dismiss()
        else:super().keyPressEvent(event)
    def mousePressEvent(self,event):event.accept()
    def mouseMoveEvent(self,event):event.accept()
    def wheelEvent(self,event):event.accept()
    def mouseReleaseEvent(self,event):
        if not self.preview.geometry().contains(event.pos()):self.dismiss()
    def paintEvent(self,event):
        p=QtGui.QPainter(self);path=QtGui.QPainterPath();path.addRoundedRect(QtCore.QRectF(self.rect()),12,12);p.fillPath(path,QtGui.QColor(5,7,11,195))

class SceneWidget(W.QWidget):
    """A genuine Qt canvas displaying an offscreen RViz render texture."""
    def __init__(self):
        super().__init__();self.process=None;self.pixels=None;self.pixel_file=None;self.image=QtGui.QImage();self.frame_seq=0;self.frame_at=0.;self.distance=0.;self.message='正在准备 3D 环境…';self.events=deque(maxlen=64);self.input_seq=0;self.previous=(0,0);self.overlay=None
        self.setMinimumHeight(420);self.setFocusPolicy(QtCore.Qt.StrongFocus);self.setMouseTracking(True);self.setObjectName('sceneHost');self.camera_focus=CameraFocus(self)
    def place_overlay(self):
        if self.overlay is not None:
            available=min(280,max(144,self.width()*.23),(self.height()-64)*16/27)
            width=max(96,int(available)//16*16);height=3*(width*9//16)+24
            self.overlay.setGeometry(20,max(20,(self.height()-height)//2),width,height);self.overlay.arrange();self.overlay.raise_()
    def resizeEvent(self,event):
        self.place_overlay();self.camera_focus.setGeometry(self.rect());
        if self.camera_focus.isVisible():self.camera_focus.raise_()
        w,h=render_size(self.width(),self.height(),self.devicePixelRatioF());atomic(ROOT/'.cache/operator/scene_viewport.json',dict(width=w,height=h));self.update()
    def image_rect(self):return fitted_rect(self.image.size(),QtCore.QRectF(self.rect()))
    def image_point(self,point):
        area=self.image_rect()
        if area.isEmpty():return (0,0)
        return (round((point.x()-area.x())*self.image.width()/area.width()),round((point.y()-area.y())*self.image.height()/area.height()))
    def start(self):
        if self.process is not None:return
        exe=ROOT/'.cache/operator/scene_build/v3_operator_scene'
        if not exe.exists():self.message='3D 渲染器尚未构建';self.update();return
        self.started=time.monotonic();self.log=(ROOT/'artifacts/operator_scene.log').open('w');atomic(ROOT/'.cache/operator/scene_input.json',[])
        self.process=subprocess.Popen(['bash','tools/operator_ros.sh',str(exe),str(ROOT)],cwd=ROOT,stdin=subprocess.DEVNULL,stdout=self.log,stderr=subprocess.STDOUT,start_new_session=True,env=dict(os.environ,QT_QPA_PLATFORM='xcb'))
    def poll(self):
        if self.process is None:return
        if self.process.poll() is not None:self.message='3D 渲染器已退出，请查看运行日志';self.update();return
        try:
            if self.pixels is None:
                self.pixel_file=(ROOT/'.cache/operator/scene_pixels.bin').open('rb');self.pixels=mmap.mmap(self.pixel_file.fileno(),0,access=mmap.ACCESS_READ)
            seq,pid,w,h,stride,stamp,distance=struct.unpack_from('<QIIIIdd',self.pixels)
            if self.frame_at and time.monotonic()-self.frame_at>1:self.message='3D 画面更新已中断';self.update()
            if seq%2 or seq==self.frame_seq or pid!=self.process.pid:return
            if not(0<w<=1600 and 0<h<=900 and stride==w*4 and 0<=time.monotonic()-stamp<1):return
            data=self.pixels[64:64+stride*h]
            if struct.unpack_from('<Q',self.pixels)[0]!=seq:return
            self.image=QtGui.QImage(data,w,h,stride,QtGui.QImage.Format_RGBA8888).copy();self.frame_seq=seq;self.frame_at=stamp;self.distance=distance;self.message='';self.update()
        except (OSError,ValueError,struct.error):pass
        if self.frame_at and time.monotonic()-self.frame_at>1:self.message='3D 画面更新已中断';self.update()
    def paintEvent(self,event):
        p=QtGui.QPainter(self);p.setRenderHints(QtGui.QPainter.Antialiasing|QtGui.QPainter.SmoothPixmapTransform);path=QtGui.QPainterPath();path.addRoundedRect(QtCore.QRectF(self.rect()),12,12);p.setClipPath(path);p.fillRect(self.rect(),QtGui.QColor('#131923'))
        if not self.image.isNull():p.drawImage(self.image_rect(),self.image)
        if self.message:
            p.setPen(QtGui.QColor('#9db3cf'));p.drawText(self.rect().adjusted(260,0,0,0),QtCore.Qt.AlignCenter,self.message)
    def send_input(self,event,kind):
        if self.image.isNull():return
        pos=event.position() if kind=='wheel' else event.localPos()
        if kind in ('press','wheel') and not self.image_rect().contains(pos):return
        x,y=self.image_point(pos);self.input_seq+=1
        if kind=='press':self.previous=(x,y)
        self.events.append(dict(seq=self.input_seq,kind=kind,x=x,y=y,lx=self.previous[0],ly=self.previous[1],button=0 if kind=='wheel' else int(event.button()),buttons=int(event.buttons()),mods=int(event.modifiers()),delta=event.angleDelta().y() if kind=='wheel' else 0));self.previous=(x,y);atomic(ROOT/'.cache/operator/scene_input.json',list(self.events));event.accept()
    def mousePressEvent(self,event):self.setFocus();self.send_input(event,'press')
    def mouseMoveEvent(self,event):
        if event.buttons():self.send_input(event,'move')
        else:self.previous=self.image_point(event.localPos())
    def mouseReleaseEvent(self,event):self.send_input(event,'release')
    def wheelEvent(self,event):self.send_input(event,'wheel')
    def stop(self):
        if self.process is not None:
            # Display-only process on its own DDS domain; no hardware connections.
            if self.process.poll() is None:self.process.terminate()
            try:self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:self.process.kill();self.process.wait()
            self.log.close();self.process=None
        if self.pixels is not None:self.pixels.close();self.pixels=None
        if self.pixel_file is not None:self.pixel_file.close();self.pixel_file=None
