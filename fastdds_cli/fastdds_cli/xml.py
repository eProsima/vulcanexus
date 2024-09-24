#!/usr/bin/env python3

from fastdds_cli.executor import Executor

def main():
    exec = Executor("xml")
    exec.execute()

if __name__ == '__main__':
    main()
