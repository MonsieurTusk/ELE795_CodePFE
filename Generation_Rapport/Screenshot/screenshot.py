import pyautogui

path = "/home/lemg/Desktop/Screenshot/img.png"
timer2500 = 2.5

# Fail-Safe, 2.5s timer
pyautogui.PAUSE = timer2500

# Avertissements - optionnel
#pyautogui.alert('alert command')
#pyautogui.confirm('confirm command')
#pyautogui.prompt('prompt command')

# Get screen size
size = pyautogui.size()

print("size, width", size.height, size.width)

# Screenshot
screenshot = pyautogui.screenshot(region=(0, 0, size.width, size.height))
screenshot.save(path)


