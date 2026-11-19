"""Generate the local operator tones on the D-drive cache, without playback."""
import math,struct,wave
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
def ensure():
    folder=ROOT/'.cache/operator/audio';folder.mkdir(parents=True,exist_ok=True)
    tones={'commissioning_ready.wav':[(880,.2),(0,.15),(1175,.3)],'cue_release.wav':[(660,.25)],'cue_joystick.wav':[(440,.15),(0,.12),(440,.15),(0,.12),(440,.15)]}
    for name,parts in tones.items():
        target=folder/name
        if target.exists():continue
        with wave.open(str(target),'wb') as w:
            w.setnchannels(1);w.setsampwidth(2);w.setframerate(44100)
            for frequency,duration in parts:
                count=int(44100*duration)
                w.writeframes(b''.join(struct.pack('<h',int(8000*min(1,i/200,(count-i)/200)*math.sin(2*math.pi*frequency*i/44100))) for i in range(count)))
if __name__=='__main__':ensure()
