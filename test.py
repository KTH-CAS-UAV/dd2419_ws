from pynput import keyboard

def on_press(key):
        print(f'Key {key.char} pressed')

def on_release(key):
    print(f'Key {key.char} released')
    if key == keyboard.Key.esc:
        # Stop listener
        return False

def main():
    listener = keyboard.Listener(
        on_press=on_press,
        on_release=on_release)
    listener.start() #on a seperate thread (non-blocking)

    while(True):
        continue

if __name__ == '__main__':
    main()