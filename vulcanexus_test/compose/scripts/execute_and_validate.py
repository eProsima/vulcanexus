# Copyright 2024 Proyectos y Sistemas de Mantenimiento SL (eProsima).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from enum import Enum
from typing import List
import argparse
import signal
import subprocess
import time

import log

import utils


DESCRIPTION = """Script to run commands and validate their output."""
USAGE = ('python3 execute_and_validate.py '
         '[-c <command>] [-t <timeout>] [-d]')


class ReturnCode(Enum):
    """Enumeration for return codes of this script."""

    SUCCESS = 0
    TIMEOUT = 1
    HARD_TIMEOUT = 2
    COMMAND_FAIL = 3
    STDERR_OUTPUT = 4


def parse_options():
    """
    Parse arguments.

    :return: The arguments parsed.
    """
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
        add_help=True,
        description=(DESCRIPTION),
        usage=(USAGE)
    )
    parser.add_argument(
        '-c',
        '--command',
        type=str,
        help='Command to execute.'
    )
    parser.add_argument(
        '-t',
        '--timeout',
        type=int,
        default=5,
        help='Timeout for the command execution.'
    )
    parser.add_argument(
        '--delay',
        type=float,
        default=0,
        help='Time to wait before starting execution.'
    )
    parser.add_argument(
        '-d',
        '--debug',
        action='store_true',
        help='Print test debugging info.'
    )

    return parser.parse_args()


def run_command(
        command: 'list[str]',
        timeout: float,
        delay: float = 0,
        timeout_as_error: bool = True):
    """
    Run command with timeout.

    :param command: Command to run in list format
    :param timeout: Timeout for the process
    :return:
        - ret_code - The process exit code
        - stdout - Output of the process
        - stderr - Error output of the process
    """
    ret_code = ReturnCode.SUCCESS

    # Delay
    utils.delay(delay)

    log.logger.debug(f'Running command: {command}')

    proc = subprocess.Popen(command,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE,
                            universal_newlines=True)

    try:
        proc.wait(timeout=timeout)

    except subprocess.TimeoutExpired:
        if timeout_as_error:
            log.logger.error(
                'Timeout expired, killing process...')
            proc.send_signal(signal.SIGINT)
            ret_code = ReturnCode.TIMEOUT

        else:
            proc.send_signal(signal.SIGINT)

    else:
        if not timeout_as_error:
            log.logger.error('Command finished before expected.')
            ret_code = ReturnCode.COMMAND_FAIL

        # Wait a minimum elapsed time to the signal to be received
        time.sleep(0.2)

    stdout, stderr = proc.communicate()

    # Check whether SIGINT was able to terminate the process
    if proc.poll() is None:
        # SIGINT couldn't terminate the process
        log.logger.error(
            'SIGINT could not kill process. '
            'Killing process hardly...')
        proc.kill()
        ret_code = ReturnCode.HARD_TIMEOUT

    log.logger.info(f'Process exited with code {proc.returncode}')

    # Set return code to COMMAND_FAIL if the process returned 1
    ret_code = ReturnCode.COMMAND_FAIL if proc.returncode == 1 else ret_code

    if not stdout:
        stdout = ''
    if not stderr:
        stderr = ''

    return (ret_code, stdout, stderr)


if __name__ == '__main__':

    # Parse arguments
    args = parse_options()

    # Set log level
    if args.debug:
        log.activate_debug()

    command = args.command.split()

    (ret_code, stdout, stderr) = run_command(
        command=command,
        timeout=args.timeout,
        delay=args.delay)

    log.logger.info(f'Command exited with code {ret_code}')

    exit(ret_code.value)
