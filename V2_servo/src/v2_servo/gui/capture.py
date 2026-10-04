"""按本窗口 HWND 截图，避免把其它覆盖窗口误作 V2 证据。仅验收使用。"""
import sys
def capture_window(root,path):
    from PIL import ImageGrab
    root.update_idletasks()
    if sys.platform=='win32':
        import ctypes
        get_ancestor=ctypes.windll.user32.GetAncestor
        get_ancestor.argtypes=[ctypes.c_void_p,ctypes.c_uint]; get_ancestor.restype=ctypes.c_void_p
        hwnd=get_ancestor(root.winfo_id(),2)
        if not hwnd: raise RuntimeError('V2 HWND unavailable')
        ImageGrab.grab(window=hwnd).save(path)
    else:
        x,y=root.winfo_rootx(),root.winfo_rooty()
        ImageGrab.grab(bbox=(x,y,x+root.winfo_width(),y+root.winfo_height())).save(path)
