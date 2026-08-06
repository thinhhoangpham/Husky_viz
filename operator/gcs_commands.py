SIMPLE = ("cancel", "teleop", "stop", "estop", "release", "auto",
          "status", "quit", "help")

def parse_command(line):
    parts = line.strip().split()
    if not parts:
        return ("noop", [])
    verb = parts[0].lower()
    if verb == "goal":
        if len(parts) != 3:
            return ("error", ["goal needs <lat> <lon>"])
        try:
            return ("goal", [float(parts[1]), float(parts[2])])
        except ValueError:
            return ("error", ["goal args must be numbers"])
    if verb in SIMPLE:
        return (verb, [])
    return ("unknown", [verb])
