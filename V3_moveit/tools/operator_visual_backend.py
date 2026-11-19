"""Choose PyOpenGL's platform before PyQtGraph imports its OpenGL helpers."""
import os,sys
from PyQt5 import QtWidgets
from operator_settings import ROOT

def prepare_opengl():
    local=str(ROOT/'.cache/operator/python')
    if local not in sys.path:sys.path.insert(0,local)
    app=QtWidgets.QApplication.instance()
    platform=app.platformName() if app else os.environ.get('QT_QPA_PLATFORM','xcb').split(':')[0]
    # START_OPERATOR selects xcb, including on a Wayland desktop. The GL binding
    # must match Qt's window/context rather than guess from WAYLAND_DISPLAY.
    os.environ.setdefault('PYOPENGL_PLATFORM','glx' if platform=='xcb' else 'egl')
