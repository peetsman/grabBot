"""Run the AVR archiver directly instead of through the hanging cmd.exe step."""

import subprocess

Import("env")


def create_archive(target, source, env):
    tool = env.WhereIs(env.subst("$AR"))
    command = [tool, *env.subst("$ARFLAGS").split(), str(target[0])]
    command.extend(str(item) for item in source)
    return subprocess.run(command, env=dict(env["ENV"]), check=False).returncode


env.Replace(ARCOM=create_archive)
