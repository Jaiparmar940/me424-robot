"""
response_router.py
==================
Parses every line coming from the ESP32 and routes it to the
appropriate handler in the bridge node.

Protocol recap (from the updated firmware)
------------------------------------------
  ACK <cmd>           — command received, execution starting
  DONE <cmd>          — command completed successfully
  ERR <cmd> <reason>  — command failed
  POS C1=N C2=N ...   — position report (from "where")
  LIMITS S2=N ...     — limit switch state
  ESTOP=N             — e-stop state
  [DBG] ...           — debug line (only when DEBUG=true on ESP32)
  [TO SLAVE] ...      — relay to MOSFET slave (informational)
  [SLAVE] ...         — response from MOSFET slave
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Optional, List, Tuple


class LineType(Enum):
    ACK    = auto()
    DONE   = auto()
    ERR    = auto()
    POS    = auto()
    LIMITS = auto()
    ESTOP  = auto()
    DEBUG  = auto()
    SLAVE  = auto()
    OTHER  = auto()


@dataclass
class ParsedLine:
    raw:      str
    kind:     LineType
    token:    str = ''          # first word after the prefix
    rest:     str = ''          # everything after the token
    # Structured fields (populated for POS and LIMITS)
    positions: List[int] = field(default_factory=list)   # [C1..C6]
    limits:    dict       = field(default_factory=dict)   # {'S2':0,'S3':0,...}
    estop:     Optional[bool] = None


def parse_line(line: str) -> ParsedLine:
    """Parse one line of ESP32 output into a structured ``ParsedLine``."""
    s = line.strip()

    if s.startswith('ACK '):
        return ParsedLine(raw=s, kind=LineType.ACK, token=s[4:].strip())

    if s.startswith('DONE '):
        return ParsedLine(raw=s, kind=LineType.DONE, token=s[5:].strip())

    if s.startswith('ERR '):
        parts = s[4:].split(' ', 1)
        token = parts[0] if parts else ''
        rest  = parts[1] if len(parts) > 1 else ''
        return ParsedLine(raw=s, kind=LineType.ERR, token=token, rest=rest)

    if s.startswith('POS '):
        pos = _parse_pos(s)
        return ParsedLine(raw=s, kind=LineType.POS, positions=pos)

    if s.startswith('LIMITS '):
        lim = _parse_limits(s)
        return ParsedLine(raw=s, kind=LineType.LIMITS, limits=lim)

    if s.startswith('ESTOP='):
        val = s[6:].strip() == '1'
        return ParsedLine(raw=s, kind=LineType.ESTOP, estop=val)

    if s.startswith('ESTOP '):
        # "ESTOP LATCHED" / "ESTOP CLEARED" from parseEStopLine
        latched = 'LATCHED' in s.upper()
        return ParsedLine(raw=s, kind=LineType.ESTOP, estop=latched)

    if s.startswith('[DBG]'):
        return ParsedLine(raw=s, kind=LineType.DEBUG, rest=s[5:].strip())

    if s.startswith('[SLAVE]') or s.startswith('[TO SLAVE]'):
        return ParsedLine(raw=s, kind=LineType.SLAVE, rest=s)

    return ParsedLine(raw=s, kind=LineType.OTHER, rest=s)


# ------------------------------------------------------------------
# Structured field parsers
# ------------------------------------------------------------------

def _parse_pos(s: str) -> List[int]:
    """
    Parse "POS C1=10 C2=0 C3=-50 C4=10 C5=0 C6=200"
    Returns a list of 6 ints [C1, C2, C3, C4, C5, C6].
    """
    positions = [0] * 6
    for i in range(1, 7):
        key = f'C{i}='
        idx = s.find(key)
        if idx < 0:
            continue
        start = idx + len(key)
        end   = start
        if end < len(s) and s[end] == '-':
            end += 1
        while end < len(s) and s[end].isdigit():
            end += 1
        try:
            positions[i - 1] = int(s[start:end])
        except ValueError:
            pass
    return positions


def _parse_limits(s: str) -> dict:
    """
    Parse "LIMITS S2=1 S3=0 S4=0 S5H=1"
    Returns {'S2': 0, 'S3': 0, 'S4': 0, 'S5H': 0} (int values).
    """
    result = {}
    for key in ('S2', 'S3', 'S4', 'S5H'):
        tag = f'{key}='
        idx = s.find(tag)
        if idx < 0:
            continue
        start = idx + len(tag)
        end   = start
        while end < len(s) and s[end].isdigit():
            end += 1
        try:
            result[key] = int(s[start:end])
        except ValueError:
            result[key] = 0
    return result
