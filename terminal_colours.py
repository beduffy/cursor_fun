class TerminalColor:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def show_colours():
    print(f"{TerminalColor.HEADER}Header{TerminalColor.ENDC}")
    print(f"{TerminalColor.OKBLUE}OKBlue{TerminalColor.ENDC}")
    print(f"{TerminalColor.OKGREEN}OKGreen{TerminalColor.ENDC}")
    print(f"{TerminalColor.WARNING}Warning{TerminalColor.ENDC}")
    print(f"{TerminalColor.FAIL}Fail{TerminalColor.ENDC}")
    print(f"{TerminalColor.BOLD}Bold{TerminalColor.ENDC}")
    print(f"{TerminalColor.UNDERLINE}Underline{TerminalColor.ENDC}")

if __name__ == "__main__":
    show_colours()
