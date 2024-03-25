#这种键盘监听是全局监听，不论是否在终端输入框中输入，键盘的任何输入都会被监听

from pynput import keyboard

def on_press(key):
    print(f'Key {key} pressed')

def on_release(key):
    if key == keyboard.Key.esc:
        # 如果按下esc键，则停止监听
        return False

# 创建监听器对象
listener = keyboard.Listener(on_press=on_press, on_release=on_release)

# 启动监听器
listener.start()

# 进入监听状态，直到按下esc键
listener.join()
