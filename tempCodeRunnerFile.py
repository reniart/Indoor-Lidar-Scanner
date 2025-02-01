    # wait for user's signal to start the program
    input("Press Enter to start scanning")
    # send the character 's' to MCU via UART
    # This will signal MCU to start the transmission
    s.write('s'.encode())