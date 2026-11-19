"""Vendor DH116 meshes and linkage; dragging emits bounded UI targets only."""
import functools,math,time
from pathlib import Path
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from PyQt5 import QtCore,QtGui,QtWidgets as W
from operator_settings import ROOT

ASSETS=ROOT/'tools/operator_assets/dh116_vendor/model'
TRAVEL=np.array([60,30,80,80,80,80],float)
PUSH_PARTS=('finger11_link','finger12_link','finger21_link','finger31_link','finger41_link','finger51_link')

def drag_tangent(projected,height,axis):
    """Bound inverse projection gain, including a nearly end-on joint view."""
    projected=np.asarray(projected,float);norm=np.linalg.norm(projected)
    direction=projected/norm if norm>=.0001 else np.array([1.,0.]) if axis==0 else np.array([0.,1.])
    minimum=np.clip(height*.75,240,640)/10000
    return direction*max(norm,minimum)

@functools.lru_cache(maxsize=1)
def model_parts():
    return yaml.safe_load((ASSETS/'model_config.yaml').read_text())['models']

def axis_for(name):
    if name=='base_link':return None
    if name=='finger11_link':return 0
    return 1 if name.startswith('finger1') else int(name[6])

def transforms(values):
    """LHandPro: additive Euler channels, per-axis gain, negative-angle ZYX matrix."""
    angles=np.asarray(values,float)*TRAVEL/10000;result={}
    for part in model_parts():
        name=Path(part['file']).stem;axis=axis_for(name)
        rotation=np.array(part['rotation'],float)
        if axis is not None:
            component='xyz'.index(part['drive_axis'])
            rotation[component]+=angles[axis]*part['drive_dir'][component]
        # See model/CONVENTIONS.md: the vendor viewer uses the opposite Euler
        # sign to scipy; drive_dir is an XYZ vector, not three multiplied gains.
        m=np.eye(4);m[:3,3]=part['position'];m[:3,:3]=Rotation.from_euler('xyz',-rotation,degrees=True).as_matrix()*part['scale']
        result[name]=result.get(part.get('parent_link'),np.eye(4))@m
    return result

@functools.lru_cache(maxsize=20)
def triangles(name):
    data=(ASSETS/name).read_bytes();count=int.from_bytes(data[80:84],'little')
    if len(data)!=84+50*count:raise ValueError('Unsupported or truncated vendor STL: '+name)
    dtype=np.dtype([('normal','<f4',(3,)),('vertices','<f4',(3,3)),('attribute','<u2')])
    return np.frombuffer(data,offset=84,count=count,dtype=dtype)['vertices'].copy()

@functools.lru_cache(maxsize=20)
def pick_geometry(name):
    mesh=triangles(name+'.STL')
    return (mesh[:,0],mesh[:,1]-mesh[:,0],mesh[:,2]-mesh[:,0],
            mesh.min(axis=(0,1)),mesh.max(axis=(0,1)))

def pick_part(origin,direction,values):
    """Intersect the visible meshes in local space, including palm occlusion."""
    nearest=float('inf');picked=None
    for name,matrix in transforms(values).items():
        inverse=np.linalg.inv(matrix)
        start=(inverse@np.r_[origin,1])[:3];ray=inverse[:3,:3]@direction
        vertices,a,b,low,high=pick_geometry(name)
        parallel=np.abs(ray)<1e-12
        if np.any(parallel & ((start<low)|(start>high))):continue
        lo=(low-start)/np.where(parallel,1,ray);hi=(high-start)/np.where(parallel,1,ray)
        entry=np.max(np.where(parallel,-np.inf,np.minimum(lo,hi)))
        leave=np.min(np.where(parallel,np.inf,np.maximum(lo,hi)))
        if leave<max(0,entry) or entry>=nearest:continue
        cross=np.cross(ray,b);det=np.einsum('ij,ij->i',a,cross)
        inv=np.divide(1.,det,out=np.zeros_like(det),where=np.abs(det)>1e-12)
        offset=start-vertices;q=np.cross(offset,a)
        u=np.einsum('ij,ij->i',offset,cross)*inv;v=np.einsum('ij,j->i',q,ray)*inv
        t=np.einsum('ij,ij->i',b,q)*inv
        hits=t[(u>=0)&(v>=0)&(u+v<=1)&(t>0)]
        if hits.size and hits.min()<nearest:nearest=float(hits.min());picked=name
    return picked if picked and axis_for(picked) is not None else None

class HandModel(W.QWidget):
    targetChanged=QtCore.pyqtSignal(int,int)
    previewChanged=QtCore.pyqtSignal(int,int)
    def __init__(self,parent=None):
        super().__init__(parent);self.values=[0]*6;self.low=[0]*6;self.high=[10000]*6;self.allowed=True;self.dragging=None;self.gl=None;self.pose_initialized=False;self.background='#ffffff';self.foreground='#1d1d1f'
        self.drag_timer=QtCore.QTimer(self);self.drag_timer.setTimerType(QtCore.Qt.PreciseTimer);self.drag_timer.setInterval(8);self.drag_timer.timeout.connect(self.advance_drag)
        self.setMinimumSize(320,330);box=W.QVBoxLayout(self);box.setContentsMargins(0,0,0,0);box.setSpacing(5)
        row=W.QHBoxLayout();self.mode=W.QPushButton('推动关节');self.mode.setCheckable(True);self.mode.setChecked(True);row.addWidget(self.mode)
        self.reset=W.QPushButton('复位视角');self.reset.clicked.connect(self.reset_camera);row.addWidget(self.reset);row.addStretch();box.addLayout(row)
        self.canvas=W.QWidget();self.canvas.setMinimumHeight(270);self.canvas_layout=W.QVBoxLayout(self.canvas);self.canvas_layout.setContentsMargins(0,0,0,0);box.addWidget(self.canvas,1)
        self.hint=W.QLabel('左键沿手指弯曲方向拖动任一指节 · 右键旋转 · 滚轮缩放');self.hint.setObjectName('muted');self.hint.setWordWrap(True);box.addWidget(self.hint)
        self.mode.toggled.connect(self.mode_changed)
    def mode_changed(self,enabled):
        if not enabled:self.cancel_drag()
        self.recolor()
    def cancel_drag(self):
        self.drag_timer.stop();self.dragging=None;self.recolor()
    def hideEvent(self,event):self.cancel_drag();super().hideEvent(event)
    def showEvent(self,event):
        super().showEvent(event)
        if self.gl is None and W.QApplication.platformName()!='offscreen':self.create_view()
    def create_view(self):
        from operator_visual_backend import prepare_opengl
        prepare_opengl()
        try:
            import pyqtgraph.opengl as gl
            from pyqtgraph.opengl.shaders import ShaderProgram,VertexShader,FragmentShader
        except ImportError as e:
            self.hint.setText('三维组件未安装：'+str(e));return
        owner=self
        if 'v3Hand' not in ShaderProgram.names:
            ShaderProgram('v3Hand',[
                VertexShader('varying vec3 n; void main(){n=normalize(gl_NormalMatrix*gl_Normal);gl_FrontColor=gl_Color;gl_BackColor=gl_Color;gl_Position=ftransform();}'),
                FragmentShader('varying vec3 n; void main(){vec3 N=normalize(n);if(!gl_FrontFacing)N=-N;vec3 L=normalize(vec3(-0.4,0.6,1.0));float diffuse=max(0.0,dot(N,L));float spec=pow(max(0.0,dot(N,normalize(L+vec3(0,0,1)))),28.0);gl_FragColor=vec4(gl_Color.rgb*(0.48+0.52*diffuse)+vec3(0.18*spec),gl_Color.a);}')])
        class View(gl.GLViewWidget):
            def resizeEvent(self,e):
                super().resizeEvent(e)
                self.opts['distance']=max(.32,.115*max(1,self.width()/max(1,self.height()))/math.tan(math.radians(20)));self.update()
            def mousePressEvent(self,e):
                if e.button()==QtCore.Qt.LeftButton:
                    if owner.mode.isChecked() and owner.allowed:
                        inverse,ok=(self.projectionMatrix()*self.viewMatrix()).inverted()
                        if ok:
                            x=2*e.x()/self.width()-1;y=1-2*e.y()/self.height()
                            points=[]
                            for z in (-1,1):
                                v=inverse*QtGui.QVector4D(x,y,z,1)
                                points.append(np.array([v.x(),v.y(),v.z()])/v.w())
                            part=pick_part(points[0],points[1]-points[0],owner.values)
                            if part:
                                owner.begin_drag(owner.items[part],e.pos());self.setCursor(QtCore.Qt.ClosedHandCursor)
                    e.accept();return
                owner.cancel_drag()
                super().mousePressEvent(e)
            def mouseMoveEvent(self,e):
                if owner.dragging is not None:
                    if e.buttons() & QtCore.Qt.LeftButton:owner.move_drag(e.pos())
                    else:owner.cancel_drag();self.unsetCursor()
                    e.accept();return
                if e.buttons()==QtCore.Qt.RightButton:
                    pos=e.localPos();delta=pos-getattr(self,'mousePos',pos);self.mousePos=pos;self.orbit(-delta.x(),delta.y());e.accept();return
                if e.buttons()==QtCore.Qt.MiddleButton:super().mouseMoveEvent(e)
                else:e.accept()
            def mouseReleaseEvent(self,e):
                if e.button()==QtCore.Qt.LeftButton:
                    if owner.dragging is not None:owner.move_drag(e.pos(),final=True);owner.cancel_drag()
                    self.unsetCursor();e.accept();return
                super().mouseReleaseEvent(e)
            def focusOutEvent(self,e):owner.cancel_drag();self.unsetCursor();super().focusOutEvent(e)
            def wheelEvent(self,e):owner.cancel_drag();super().wheelEvent(e)
        self.gl=View();self.canvas_layout.addWidget(self.gl);self.items={}
        for part in model_parts():
            name=Path(part['file']).stem;v=triangles(Path(part['file']).name)
            item=gl.GLMeshItem(vertexes=v,smooth=False,shader='v3Hand',color=QtGui.QColor(part['color']).getRgbF(),glOptions='opaque')
            item.axis_index=axis_for(name);item.part_name=name;item.local_center=np.mean(v.reshape(-1,3),axis=0);self.items[name]=item;self.gl.addItem(item)
        self.reset_camera();self.set_theme(self.background,self.foreground);self.set_values(self.values);self.recolor()
    def reset_camera(self):
        if self.gl:
            distance=max(.32,.115*max(1,self.gl.width()/max(1,self.gl.height()))/math.tan(math.radians(20)))
            self.gl.opts['center']=QtGui.QVector3D(0,0,.09);self.gl.setCameraPosition(distance=distance,elevation=12,azimuth=-25);self.gl.opts['fov']=40;self.gl.update()
    def set_theme(self,bg,fg):
        self.background=bg;self.foreground=fg
        if self.gl:self.gl.setBackgroundColor(bg)
    def set_values(self,values):
        if self.dragging is not None:return
        values=[int(v) for v in values]
        if len(values)!=6 or (values==self.values and self.pose_initialized):return
        self.values=values;self.update_pose()
    def update_pose(self):
        if not self.gl:return
        for name,m in transforms(self.values).items():self.items[name].setTransform(QtGui.QMatrix4x4(*m.ravel()))
        self.pose_initialized=True;self.gl.update()
    def recolor(self):
        if not self.gl:return
        for part in model_parts():
            name=Path(part['file']).stem;item=self.items[name];color=part['color']
            if name in PUSH_PARTS and self.mode.isChecked():color='#d5aca4'
            if self.dragging is not None and item.axis_index==self.dragging['axis']:color='#68a4e8' if part['color']!='#2A2A2A' else '#3a526b'
            item.setColor(QtGui.QColor(color).getRgbF())
    def screen_point(self,name,local,values):
        world=transforms(values)[name]@np.r_[local,1]
        v=(self.gl.projectionMatrix()*self.gl.viewMatrix())*QtGui.QVector4D(*world)
        return np.array([(v.x()/v.w()+1)*self.gl.width()/2,(1-v.y()/v.w())*self.gl.height()/2])
    def begin_drag(self,item,pos):
        axis=item.axis_index;base=self.screen_point(item.part_name,item.local_center,self.values);after=list(self.values);after[axis]+=100
        tangent=(self.screen_point(item.part_name,item.local_center,after)-base)/100
        tangent=drag_tangent(tangent,self.gl.height(),axis);now=time.monotonic()
        self.dragging=dict(axis=axis,start=np.array([pos.x(),pos.y()]),value=self.values[axis],wanted=self.values[axis],tangent=tangent,last_tick=now,last_send=now,last_value=round(self.values[axis]),last_preview=round(self.values[axis]));self.recolor()
    def move_drag(self,pos,final=False):
        if self.dragging is None or not self.allowed:return
        d=self.dragging;axis=d['axis'];delta=np.array([pos.x(),pos.y()])-d['start'];t=d['tangent']
        d['wanted']=float(np.clip(d['value']+np.dot(delta,t)/np.dot(t,t),self.low[axis],self.high[axis]))
        if final:self.advance_drag(final=True)
        elif not self.drag_timer.isActive():self.drag_timer.start()
    def advance_drag(self,final=False):
        if self.dragging is None or not self.allowed:self.drag_timer.stop();return
        d=self.dragging;axis=d['axis'];now=time.monotonic();dt=max(0,now-d['last_tick']);d['last_tick']=now
        wanted=float(np.clip(d['wanted'],self.low[axis],self.high[axis]))
        # Display is independent of the bounded 30 Hz motor-command stream.
        # A short continuous response removes pointer-event steps without overshoot.
        value=wanted if final else self.values[axis]+(wanted-self.values[axis])*(-math.expm1(-dt/.018))
        settled=abs(value-wanted)<.05
        if settled:value=wanted
        self.values[axis]=value;self.update_pose();rounded=round(value)
        if rounded!=d['last_preview']:d['last_preview']=rounded;self.previewChanged.emit(axis,rounded)
        if rounded!=d['last_value'] and (final or now-d['last_send']>=1/30):
            d['last_value']=rounded;d['last_send']=now;self.targetChanged.emit(axis,rounded)
        if settled and rounded==d['last_value']:self.drag_timer.stop()
    def set_allowed(self,allowed):
        if self.allowed==allowed:return
        self.allowed=allowed
        if not allowed:self.cancel_drag()
