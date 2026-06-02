import scservo_sdk as scs

ph = scs.PortHandler('/dev/ttyACM0')
ph.openPort()
ph.setBaudRate(1000000)
pkh = scs.PacketHandler(0)

for motor_id in range(1, 7):
    val, comm, err = pkh.read2ByteTxRx(ph, motor_id, 56)
    status = "OK" if comm == 0 else "NO RESPONDE"
    print(f"Motor ID {motor_id}: {status} (val={val})")

ph.closePort()
