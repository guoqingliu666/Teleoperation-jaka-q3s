"""Measured discrete taxels on a registered illustration, not a contact-shape estimator."""
import functools,math
import numpy as np
from PyQt5 import QtCore,QtGui,QtWidgets as W
from operator_settings import ROOT
from operator_tactile_plot import resultant

ASSETS=ROOT/'tools/operator_assets/dh116_vendor'
FINGERS=('大拇指','食指','中指','无名指','小拇指')
# Illustration registration only. SDK normalized channel positions are authoritative;
# these quads are explicitly not a physical millimetre calibration.
QUADS=[[(395,340),(470,313),(492,360),(416,388)],[(274,34),(312,36),(300,234),(266,230)],[(211,16),(249,16),(242,232),(207,232)],[(141,25),(179,25),(183,235),(147,239)],[(79,67),(112,65),(126,226),(90,233)]]

def registered(index,point):
    q=np.asarray(QUADS[index],float);x,y=point
    return (1-y)*((1-x)*q[0]+x*q[1])+y*((1-x)*q[3]+x*q[2])

@functools.lru_cache(maxsize=12)
def cells(positions):
    """Nearest-channel display regions, capped around sampled channel centres."""
    result=[]
    for i,p in enumerate(positions):
        p=np.array(p);x,y=p;poly=[np.array(v) for v in [(max(0,x-.15),max(0,y-.12)),(min(1,x+.15),max(0,y-.12)),(min(1,x+.15),min(1,y+.12)),(max(0,x-.15),min(1,y+.12))]]
        for j,q in enumerate(positions):
            if i==j:continue
            q=np.array(q);normal=q-p;limit=(q@q-p@p)/2;out=[]
            for a,b in zip(poly,poly[1:]+poly[:1]):
                da=a@normal-limit;db=b@normal-limit
                if da<=1e-9:out.append(a)
                if (da<=0)!=(db<=0):out.append(a+(b-a)*da/(da-db))
            poly=out
            if not poly:break
        result.append(poly)
    return result

class PressureHand(W.QWidget):
    def __init__(self,rich=False,compact=False):
        super().__init__();self.image=QtGui.QPixmap(str(ASSETS/'hand_116.png'));self.state={};self.heat=True;self.arrows=True;self.scale_n=3;self.threshold=.03;self.background='#ffffff';self.foreground='#1d1d1f';self.hit_regions=[]
        self.setObjectName('handCanvas');self.setMinimumSize(190,170);self.setMouseTracking(True)
    def set_state(self,state):self.state=state;self.update()
    def paintEvent(self,event):
        p=QtGui.QPainter(self);p.setRenderHints(QtGui.QPainter.Antialiasing|QtGui.QPainter.SmoothPixmapTransform);p.fillRect(self.rect(),QtGui.QColor(self.background))
        scale=min((self.width()-12)/517,(self.height()-8)/580);ox=(self.width()-517*scale)/2;oy=(self.height()-580*scale)/2;p.translate(ox,oy);p.scale(scale,scale);p.drawPixmap(0,0,self.image);self.hit_regions=[]
        for finger,f in enumerate(self.state.get('fingers',[])):
            values=f.get('pressure',[]);xy=f.get('pressure_xy',[])
            if not values or len(values)!=len(xy) or f.get('pressure_unit')!='normalized':continue
            polygons=cells(tuple(tuple(v) for v in xy))
            for j,(value,poly) in enumerate(zip(values,polygons)):
                points=[registered(finger,v) for v in poly];shape=QtGui.QPolygonF([QtCore.QPointF(*v) for v in points])
                screen=QtGui.QPolygonF([QtCore.QPointF(ox+x*scale,oy+y*scale) for x,y in points]);self.hit_regions.append((screen,finger,j,value,xy[j]))
                if value<=self.threshold or not self.heat:continue
                color=QtGui.QColor.fromHsvF((1-min(1,value))*.60,.85,1,.68)
                p.setPen(QtGui.QPen(QtGui.QColor(255,255,255,175),1));p.setBrush(color);p.drawPolygon(shape)
                center=registered(finger,xy[j]);p.setPen(QtCore.Qt.NoPen);p.setBrush(QtGui.QColor(255,255,255,220));p.drawEllipse(QtCore.QPointF(*center),2,2)
            active=np.array(values)>self.threshold
            if not active.any() or not self.arrows:continue
            v,valid=resultant(f)
            if not valid or np.linalg.norm(v)<.01:continue
            # Weighted centre is a taxel-intensity centroid, not a measured force location.
            weights=np.where(active,values,0);center=registered(finger,np.average(xy,axis=0,weights=weights))
            # Oblique local basis keeps a normal-only force visible in this 2-D view.
            vector=np.array([v[0]+.42*v[2],-v[1]-.70*v[2]]);norm=np.linalg.norm(vector)
            if norm<1e-8:continue
            direction=vector/norm;length=15+min(65,50*np.linalg.norm(v)/max(.1,self.scale_n));a=center;b=a+direction*length
            p.setPen(QtGui.QPen(QtGui.QColor(20,30,50,105),6,QtCore.Qt.SolidLine,QtCore.Qt.RoundCap));p.drawLine(QtCore.QPointF(*a),QtCore.QPointF(*b))
            p.setPen(QtGui.QPen(QtGui.QColor('#ff9f0a'),3,QtCore.Qt.SolidLine,QtCore.Qt.RoundCap));p.drawLine(QtCore.QPointF(*a),QtCore.QPointF(*b))
            perp=np.array([-direction[1],direction[0]]);p.setBrush(QtGui.QColor('#ff9f0a'));p.setPen(QtCore.Qt.NoPen);p.drawPolygon(QtGui.QPolygonF([QtCore.QPointF(*v) for v in [b,b-direction*10+perp*5,b-direction*10-perp*5]]))
        p.end()
    def mouseMoveEvent(self,event):
        for poly,finger,index,value,xy in self.hit_regions:
            if poly.containsPoint(QtCore.QPointF(event.pos()),QtCore.Qt.OddEvenFill):
                self.setToolTip(f'{FINGERS[finger]} · 通道 {index+1} · {value*100:.1f}%\nSDK 局部坐标 ({xy[0]:.3f}, {xy[1]:.3f})\n分区表示离散通道，不代表真实接触轮廓。');return
        self.setToolTip('厂家通道坐标映射；图上位置未做毫米标定。合力箭头是传感器局部坐标的斜投影。')
