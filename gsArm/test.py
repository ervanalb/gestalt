import serial
import struct

crcTable = [0, 7, 14, 9, 28, 27, 18, 21, 56, 63, 54, 49, 36, 35, 42, 45, 112, 119,
            126, 121, 108, 107, 98, 101, 72, 79, 70, 65, 84, 83, 90, 93, 224, 231,
            238, 233, 252, 251, 242, 245, 216, 223, 214, 209, 196, 195, 202, 205,
            144, 151, 158, 153, 140, 139, 130, 133, 168, 175, 166, 161, 180, 179,
            186, 189, 199, 192, 201, 206, 219, 220, 213, 210, 255, 248, 241, 246,
            227, 228, 237, 234, 183, 176, 185, 190, 171, 172, 165, 162, 143, 136,
            129, 134, 147, 148, 157, 154, 39, 32, 41, 46, 59, 60, 53, 50, 31, 24,
            17, 22, 3, 4, 13, 10, 87, 80, 89, 94, 75, 76, 69, 66, 111, 104, 97, 102,
            115, 116, 125, 122, 137, 142, 135, 128, 149, 146, 155, 156, 177, 182,
            191, 184, 173, 170, 163, 164, 249, 254, 247, 240, 229, 226, 235, 236,
            193, 198, 207, 200, 221, 218, 211, 212, 105, 110, 103, 96, 117, 114,
            123, 124, 81, 86, 95, 88, 77, 74, 67, 68, 25, 30, 23, 16, 5, 2, 11, 12,
            33, 38, 47, 40, 61, 58, 51, 52, 78, 73, 64, 71, 82, 85, 92, 91, 118,
            113, 120, 127, 106, 109, 100, 99, 62, 57, 48, 55, 34, 37, 44, 43, 6, 1,
            8, 15, 26, 29, 20, 19, 174, 169, 160, 167, 178, 181, 188, 187, 150, 145,
            152, 159, 138, 141, 132, 131, 222, 217, 208, 215, 194, 197, 204, 203,
            230, 225, 232, 239, 250, 253, 244, 243]

def doCrc(msg):
    crc = 0
    for m in msg:
        crc ^= ord(m)
        crc = crcTable[crc]
    return chr(crc)

s = serial.Serial("/dev/ttyUSB0", 115200, timeout = 1)

def sendPacket(port, message = "", address = None):
    if address is None:
        address = 0
        first = 138
    else:
        first = 72
    length = len(message) + 5
    message = struct.pack("<BHBB", first, address, port, length) + message
    message = message + doCrc(message)
    s.write(message)

def recvPacket(port, address = None):
    header = s.read(5)
    if len(header) < 5:
        raise Exception("Timeout")
    (first, receivedAddress, receivedPort, length) = struct.unpack("<BHBB", header)
    if address is None:
        if first != 138:
            raise Exception("Bad first byte. Expected 138, got {0}".format(ord(first)))
    else:
        if first != 72:
            raise Exception("Bad first byte. Expected 72, got {0}".format(ord(first)))
        if receivedAddress != address:
            raise Exception("Bad address. Got {0}, expected {1}.".format(receivedAddress, address))
    if port != receivedPort:
        raise Exception("Bad port. Got {0}, expected {1}.".format(receivedPort, port))
    payload = s.read(length - 5)
    if len(payload) < length - 5:
        raise Exception("Timeout")
    receivedCRC = s.read(1)
    if len(receivedCRC) < 1:
        raise Exception("Timeout")
    correctCRC = doCrc(header + payload)
    if receivedCRC != correctCRC:
        raise Exception("Bad CRC. Got {0}, expected {1}".format(ord(receivedCRC), ord(correctCRC)))
    return payload

def assertPacket(port, address = None):
    response = recvPacket(port, address)
    if response != "":
        raise Exception("Received unexpected response: {0}".format(repr(response)))

def send_svcRequestURL(address = None):
    sendPacket(5, address = address)
    return recvPacket(5, address = address)

def send_MoveTo(x, y, z, t, address = None):
    sendPacket(10, struct.pack("<iiii", int(x * 65536), int(y * 65536), int (z * 65536), int(t * 65536)), address = address)

def send_Jog(xv, yv, zv, t, address = None):
    sendPacket(13, struct.pack("<iiii", int(xv * 65536), int(yv * 65536), int (zv * 65536), int(t * 65536)), address = address)
    assertPacket(13, address = address)

def send_SetCurrent(xyc, zc, address = None):
    sendPacket(11, struct.pack("<HH", int(xyc * 65535), int(zc * 65535)), address = address)
    assertPacket(11, address = address)

def send_Zero(x, y, z, address = None):
    sendPacket(12, struct.pack("<iii", int(x * 65536), int(y * 65536), int(z*65536)), address = address)
    assertPacket(12, address = address)

def send_GetPosition(address = None):
    sendPacket(14, "", address = address)
    response = recvPacket(14, address = address)
    (xf, yf, zf) = struct.unpack("<iii", response)
    return (float(xf)/65536, float(yf)/65536, float(zf)/65536)

def send_GetButtons(address = None):
    sendPacket(15, "", address = address)
    response = recvPacket(15, address = address)
    (lb, rb) = struct.unpack("<BB", response)
    return tuple([bool(b) for b in (lb,rb)])

import time

print send_svcRequestURL()
send_SetCurrent(1, 1)
send_Zero(0, 0, 0)
send_MoveTo(400, 0, 0, 2)
time.sleep(3)
send_MoveTo(0, 0, 0, 1)

while True:
    print send_GetButtons()
    time.sleep(0.1)
