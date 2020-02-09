from contextlib import contextmanager
import os
import sys


@contextmanager
def ignore_stderr(enable=True):
    """
    Suppress error messages from ALSA
    https://stackoverflow.com/questions/7088672/pyaudio-working-but-spits-out-error-messages-each-time
    https://stackoverflow.com/questions/36956083/how-can-the-terminal-output-of-executables-run-by-python-functions-be-silenced-i
    :param enable:
    """
    if enable:
        devnull = None
        try:
            devnull = os.open(os.devnull, os.O_WRONLY)
            stderr = os.dup(2)
            sys.stderr.flush()
            os.dup2(devnull, 2)
            try:
                yield
            finally:
                os.dup2(stderr, 2)
                os.close(stderr)
        finally:
            if devnull is not None:
                os.close(devnull)
    else:
        yield
