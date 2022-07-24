import time


class Logger:

    def __init__(self, path: str):
        self.file = open("../logs/" + path + ".simlog", 'w')

    def log(self, what: str) -> None:
        now = round(time.time()*1000)
        line = str(now) + ", " + what + "\n"
        self.file.write(line)

    def finish(self) -> None:
        self.file.close()
