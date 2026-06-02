import scservo_sdk as scs

ph = scs.PortHandler('/dev/ttyACM1')
ph.openPort()
ph.setBaudRate(1000000)
pkh = scs.PacketHandler(0)
val, comm, err = pkh.read2ByteTxRx(ph, 1, 56)
print('value:', val, 'comm:', comm, pkh.getTxRxResult(comm))
ph.closePort()
