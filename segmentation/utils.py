import argparse
import datetime
import os


def create_dir(dir: str, name: str) -> str:
    date = datetime.datetime.now().strftime('%d_%m_%Y-%H_%M')
    directory = os.path.join(dir, f'{name}_{date}')
    os.makedirs(directory, exist_ok=True)
    return directory


def bool_flag(s):
    """
    Parse boolean arguments from the command line.
    """
    false = {"off", "false", "0"}
    true = {"on", "true", "1"}
    if s.lower() in false:
        return False
    elif s.lower() in true:
        return True
    else:
        raise argparse.ArgumentTypeError("invalid value for a boolean flag")
