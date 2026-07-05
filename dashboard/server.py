import asyncio
import serial
import websockets
import json

PORT = "COM6"
BAUDRATE = 115200

def parse_line(line):
    """Parse 'VIN:12000 VOUT:5000 TARGET:6000 DUTY:165'"""
    data = {}
    try:
        parts = line.strip().split()
        for part in parts:
            if ":" in part:
                key, val = part.split(":", 1)
                data[key] = int(val)
    except:
        pass
    return data

async def handler(websocket):
    print(f"[WS] Client connecté : {websocket.remote_address}")
    try:
        ser = serial.Serial(PORT, BAUDRATE, timeout=0.1)
        print(f"[SERIAL] Port {PORT} ouvert")
    except Exception as e:
        print(f"[SERIAL] Erreur ouverture {PORT} : {e}")
        await websocket.send(json.dumps({"error": f"Impossible d'ouvrir {PORT}"}))
        return

    async def read_serial():
        loop = asyncio.get_event_loop()
        while True:
            try:
                line = await loop.run_in_executor(None, ser.readline)
                line = line.decode("utf-8", errors="ignore").strip()
                if line:
                    data = parse_line(line)
                    if data:
                        await websocket.send(json.dumps(data))
            except Exception as e:
                print(f"[SERIAL] Erreur lecture : {e}")
                break

    async def read_ws():
        async for message in websocket:
            try:
                print(f"[WS] Reçu : {message}")
                ser.write((message + "\r\n").encode("utf-8"))
            except Exception as e:
                print(f"[SERIAL] Erreur écriture : {e}")
                break

    try:
        await asyncio.gather(read_serial(), read_ws())
    finally:
        ser.close()
        print(f"[SERIAL] Port {PORT} fermé")

async def main():
    print("=" * 40)
    print("  STM32 Dashboard Backend")
    print(f"  Port série : {PORT}")
    print(f"  WebSocket  : ws://localhost:8765")
    print("=" * 40)
    async with websockets.serve(handler, "localhost", 8765):
        await asyncio.Future()

if __name__ == "__main__":
    asyncio.run(main())