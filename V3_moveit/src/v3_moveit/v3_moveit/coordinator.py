"""慢规划结果的身份检查。目标变化、场景变化、松手均使旧结果作废。"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class Ticket:
    generation: int
    scene: int
    start: tuple


class Epoch:
    def __init__(self): self.generation=0; self.scene=0
    def invalidate(self): self.generation+=1
    def scene_changed(self): self.scene+=1; self.invalidate()
    def ticket(self, q): return Ticket(self.generation,self.scene,tuple(q))
    def accepts(self, ticket, actual_q, held):
        return (held and ticket.generation==self.generation and ticket.scene==self.scene
                and np.max(np.abs(np.array(ticket.start)-np.array(actual_q)))<.01)
