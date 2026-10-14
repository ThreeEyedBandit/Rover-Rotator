# Current SCons passes a non-string into the command printer and aborts
# while writing firmware.bin. Print only real command strings.
Import("env")


def _print_cmd_line(command, target, source, env):
    if isinstance(command, str) and command:
        print(command)


env["PRINT_CMD_LINE_FUNC"] = _print_cmd_line
