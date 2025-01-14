class SystemElement:
    def __init__(self, system, index: int):
        self._system = system
        self._index = index

    @property
    def index(self) -> int:
        return self._index

    @property
    def system(self):
        return self._system

    def __repr__(self) -> str:
        return f"{self.__class__.__name__} {self._index}"
