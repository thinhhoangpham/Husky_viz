SIMPLE = ("cancel", "teleop", "stop", "estop", "release", "auto",
          "status", "quit", "help")

def parse_command(line):
    parts = line.strip().split()
    if not parts:
        return ("noop", [])
    verb = parts[0].lower()
    if verb == "goal":
        rest = parts[1:]
        # goal xy <x> <y>  -- map-frame metres
        if rest and rest[0] == "xy":
            if len(rest) != 3:
                return ("error", ["goal xy needs <x> <y>"])
            try:
                return ("goal_xy", [float(rest[1]), float(rest[2])])
            except ValueError:
                return ("error", ["goal xy args must be numbers"])
        # goal <lat> <lon>  -- two numeric args (existing behaviour)
        if len(rest) == 2:
            try:
                return ("goal", [float(rest[0]), float(rest[1])])
            except ValueError:
                return ("error", ["goal args must be numbers"])
        # goal <name>  -- single non-numeric arg -> named place lookup
        if len(rest) == 1:
            return ("goal_name", [rest[0]])
        return ("error", ["goal needs <lat> <lon>, xy <x> <y>, or <name>"])
    if verb == "mode":
        rest = parts[1:]
        if len(rest) != 1:
            return ("error", ["mode needs <gps|landmark>"])
        val = rest[0].lower()
        if val not in ("gps", "landmark"):
            return ("error", ["mode must be gps or landmark"])
        return ("mode", [val])
    if verb in SIMPLE:
        return (verb, [])
    return ("unknown", [verb])
