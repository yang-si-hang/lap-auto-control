import select
import sys
import termios
import tty

# set up the terminal to read keyboard input without blocking
def get_char_keyboard():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return None

while True:
    # your code inside the infinite loop
    print("looping")
    # check if any key is pressed
    c = get_char_keyboard()
    if c is not None:
        if c == 'q':
            print("Stopping infinite loop...")
            break
