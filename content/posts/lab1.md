+++
title = "Lab1: Artemis and Bluetooth"
date = "2026-02-03"
+++

# Lab 1A

```cpp
case ECHO:

    char char_arr[MAX_MSG_SIZE];

    // Extract the next value from the command string as a character array
    success = robot_cmd.get_next_value(char_arr);
    if (!success)
        return;

    tx_estring_value.clear();
    tx_estring_value.append(char_arr);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());

    Serial.print("Robot says -> ");
    Serial.print(tx_estring_value.c_str());
    Serial.println(":)");
    
    break;
```

```python
ble.send_command(CMD.ECHO, "HiHello")
```

{{ image(path="content/posts/lab1/serial_print.png", alt="serial screenshot", width=600, class="center" )}}

<video controls>
  <source src="content/posts/lab1/blink.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>