# "지금 무엇을 할 차례인가"를 정하는 상태기계와, 상태에 맞는 동작 순서를 실행하는 제어기
"""상태 5개:  idle(대기) → picking(집기) → flipping(뒤집기) 또는 placing(놓기) → idle … → done(완료)

뒤집기를 마치면 다시 picking으로 돌아가 뒤집힌 박스를 한 번 더 집는다.
"""

from dataclasses import replace

from transitions import Machine

from palletizing.tasks import Command, Observation, Task, flip_sequence, pick_sequence, place_sequence


class PalletizerFSM:
    STATES = ["idle", "picking", "flipping", "placing", "done"]

    # trigger: 일어난 일, source → dest: 상태 이동, conditions/unless: 이 함수가 참/거짓일 때만
    TRANSITIONS = [
        {"trigger": "go_pick", "source": "idle", "dest": "picking"},
        {"trigger": "pick_done", "source": "picking", "dest": "flipping", "conditions": "needs_flip"},
        {"trigger": "pick_done", "source": "picking", "dest": "placing", "unless": "needs_flip"},
        {"trigger": "flip_done", "source": "flipping", "dest": "picking"},
        {"trigger": "place_done", "source": "placing", "dest": "done", "conditions": "last_bin"},
        {"trigger": "place_done", "source": "placing", "dest": "idle", "unless": "last_bin"},
    ]

    def __init__(self):
        self._obs: Observation | None = None
        # Machine이 self에 state 속성과 go_pick() 같은 trigger 메서드를 붙여 준다.
        self.machine = Machine(
            model=self, states=self.STATES, transitions=self.TRANSITIONS, initial="idle", after_state_change="_log"
        )

    def step(self, obs: Observation, sequence_done: bool = False) -> tuple[bool, bool]:
        """관측을 보고 상태를 옮긴다. (상태가 바뀌었는가, 박스 하나를 쌓았는가)를 돌려준다."""
        self._obs = obs
        before = self.state
        stacked = False
        if self.state == "idle":
            if obs.has_active_bin and not obs.bin_attached:
                self.go_pick()
        elif self.state != "done" and not obs.has_active_bin:
            print("<박스를 놓쳤다. 대기로 돌아간다>")
            self.reset()
        elif sequence_done:
            if self.state == "picking":
                self.pick_done()
            elif self.state == "flipping":
                self.flip_done()
            elif obs.bin_at_stack_target:
                self.place_done()
                stacked = True
            else:
                print("<놓을 자리를 벗어났다. 대기로 돌아간다>")
                self.reset()
        return self.state != before, stacked

    def reset(self) -> None:
        self.machine.set_state("idle")

    # 조건 함수. TRANSITIONS의 conditions/unless에서 이름으로 불린다.
    def needs_flip(self) -> bool:
        return self._obs.has_active_bin and self._obs.bin_needs_flip

    def last_bin(self) -> bool:
        return self._obs.last_bin

    def _log(self) -> None:
        print(f"  -> {self.state}")


class PalletizerController:
    """상태기계가 고른 상태의 동작 순서를 한 조각씩 실행한다."""

    def __init__(self):
        self.fsm = PalletizerFSM()
        self.reset()

    @property
    def state(self) -> str:
        return self.fsm.state

    def reset(self) -> None:
        self.fsm.reset()
        self._sequence: list[Task] = []
        self._index = 0
        self._after_flip = False

    def step(self, obs: Observation) -> Command:
        changed, _ = self.fsm.step(obs)
        if changed:
            self._load_sequence(obs)
        if not self._sequence:
            return Command()

        done, command = self._sequence[self._index].step(obs)
        if not done:
            return command

        # 조각 하나가 끝났다. 다음 조각으로, 순서가 다 끝났으면 상태기계에 알린다.
        self._index += 1
        if self._index < len(self._sequence):
            self._sequence[self._index].enter(obs)
            return command
        was_flipping = self.fsm.state == "flipping"
        _, stacked = self.fsm.step(obs, sequence_done=True)
        self._after_flip = was_flipping and self.fsm.state == "picking"
        self._load_sequence(obs)
        return replace(command, complete_bin=stacked)

    def _load_sequence(self, obs: Observation) -> None:
        if self.fsm.state == "picking":
            self._sequence = pick_sequence(obs, after_flip=self._after_flip)
        elif self.fsm.state == "flipping":
            self._sequence = flip_sequence()
        elif self.fsm.state == "placing":
            self._sequence = place_sequence()
        else:
            self._sequence = []
        self._index = 0
        if self._sequence:
            self._sequence[0].enter(obs)
