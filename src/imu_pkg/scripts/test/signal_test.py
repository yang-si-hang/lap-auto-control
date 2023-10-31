import signal

# define a signal handler function
def signal_handler(sig, frame):
    # print('Stopping infinite loop...')
    raise KeyboardInterrupt

# set up the signal handler for SIGINT (Ctrl+C)
signal.signal(signal.SIGINT, signal_handler)

try:
    while True:
        # your code inside the infinite loop
        print("loop")
        # add any other conditions to exit the loop if necessary

except KeyboardInterrupt:
    # handle the KeyboardInterrupt exception
    print('Stopping infinite loop...')

finally:
    # your code after the loop
    print('Operations after the loop')
