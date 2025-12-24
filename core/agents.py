from __future__ import annotations

import random
import heapq
from dataclasses import dataclass
from typing import Callable

from core.grid import Grid
from core.path import Path


@dataclass(slots=True)
class Agent:
    name: str

    def find_path(self, grid: Grid, start: tuple[int, int], goal: tuple[int, int]) -> Path:
        raise NotImplementedError


class ExampleAgent(Agent):

    def __init__(self):
        super().__init__("Example")

    def find_path(self, grid: Grid, start: tuple[int, int], goal: tuple[int, int]) -> Path:
        nodes = [start]
        while nodes[-1] != goal:
            r, c = nodes[-1]
            neighbors = grid.neighbors4(r, c)

            min_dist = min(grid.manhattan(t.pos, goal) for t in neighbors)
            best_tiles = [
                tile for tile in neighbors
                if grid.manhattan(tile.pos, goal) == min_dist
            ]
            best_tile = best_tiles[random.randint(0, len(best_tiles) - 1)]

            nodes.append(best_tile.pos)

        return Path(nodes)


class DFSAgent(Agent):

    def __init__(self):
        super().__init__("DFS")

    def find_path(self, grid: Grid, start: tuple[int, int], goal: tuple[int, int]) -> Path:
        stek = [start]
        poseceni = set()
        roditelj = {start: None}


        prioritet_smera = {
            (0, 1): 0,     # istok
            (1, 0): 1,     # jug
            (0, -1): 2,    # zapad
            (-1, 0): 3     # sever
        }

        while stek:
            trenutno = stek.pop()

            if trenutno in poseceni:
                continue
            poseceni.add(trenutno)

            if trenutno == goal:
                return self._napravi_put(roditelj, goal)

            r, c = trenutno
            susedi = grid.neighbors4(r, c)


            susedi.sort(
                key=lambda polje: (
                    polje.cost,
                    prioritet_smera[
                        (polje.pos[0] - r, polje.pos[1] - c)
                    ]
                )
            )


            for polje in reversed(susedi):
                if polje.pos not in poseceni:
                    roditelj[polje.pos] = trenutno
                    stek.append(polje.pos)

        return Path([])

    def _napravi_put(self, roditelj, cilj) -> Path:
        put = []
        tekuci = cilj
        while tekuci is not None:
            put.append(tekuci)
            tekuci = roditelj[tekuci]
        put.reverse()
        return Path(put)


class BranchAndBoundAgent(Agent):

    def __init__(self):
        super().__init__("BranchAndBound")

    def find_path(self, grid: Grid, start: tuple[int, int], goal: tuple[int, int]) -> Path:
        red = []
        heapq.heappush(red, (0, 0, start))   # (cena, duzina, pozicija)

        roditelj = {start: None}
        najbolja_cena = {start: 0}

        while red:
            cena, duzina, trenutno = heapq.heappop(red)

            if trenutno == goal:
                return self._napravi_put(roditelj, goal)

            r, c = trenutno
            for polje in grid.neighbors4(r, c):
                nova_cena = cena + polje.cost

                if polje.pos not in najbolja_cena or nova_cena < najbolja_cena[polje.pos]:
                    najbolja_cena[polje.pos] = nova_cena
                    roditelj[polje.pos] = trenutno
                    heapq.heappush(red, (nova_cena, duzina + 1, polje.pos))

        return Path([])

    def _napravi_put(self, roditelj, cilj) -> Path:
        put = []
        tekuci = cilj
        while tekuci is not None:
            put.append(tekuci)
            tekuci = roditelj[tekuci]
        put.reverse()
        return Path(put)


class AStar(Agent):

    def __init__(self):
        super().__init__("AStar")

    def find_path(self, grid: Grid, start: tuple[int, int], goal: tuple[int, int]) -> Path:
        red = []
        heapq.heappush(red, (0, 0, start))   # (f, duzina, pozicija)

        roditelj = {start: None}
        g_vrednost = {start: 0}


        while red:
            _, duzina, trenutno = heapq.heappop(red)

            if trenutno == goal:
                return self._napravi_put(roditelj, goal)

            r, c = trenutno
            for polje in grid.neighbors4(r, c):
                nova_g = g_vrednost[trenutno] + polje.cost

                if polje.pos not in g_vrednost or nova_g < g_vrednost[polje.pos]:
                    g_vrednost[polje.pos] = nova_g
                    f = nova_g + grid.manhattan(polje.pos, goal)  #f(n) = g(n) + h(n)
                    roditelj[polje.pos] = trenutno
                    heapq.heappush(red, (f, duzina + 1, polje.pos))

        return Path([])

    def _napravi_put(self, roditelj, cilj) -> Path:
        put = []
        tekuci = cilj
        while tekuci is not None:
            put.append(tekuci)
            tekuci = roditelj[tekuci]
        put.reverse()
        return Path(put)


AGENTS: dict[str, Callable[[],  Agent]] = {
    "Example": ExampleAgent,
    "DFS": DFSAgent,
    "BranchAndBound": BranchAndBoundAgent,
    "AStar": AStar
}


def create_agent(name: str) -> Agent:
    if name not in AGENTS:
        raise ValueError(f"Unknown agent '{name}'. Available: {', '.join(AGENTS.keys())}")
    return AGENTS[name]()


#py main.py maps/map0.txt DFS