import asyncio
import time
from bleak import BleakScanner, BleakClient

# ===================== Settings =====================

DEVICE_NAME = "PET_TRACKER"
SERVICE_UUID = "19B10000-E8F2-537E-4F6C-D104768A1214".lower()
RX_UUID = "19B10001-E8F2-537E-4F6C-D104768A1214"
TX_UUID = "19B10002-E8F2-537E-4F6C-D104768A1214"

SCAN_TIMEOUT = 10.0
COMMAND_RESPONSE_DELAY = 0.3

# ====================================================


async def find_device():
    print(f"[PY] Searching for BLE device '{DEVICE_NAME}' by service UUID...")
    device = await BleakScanner.find_device_by_filter(
        lambda d, adv: SERVICE_UUID in [u.lower() for u in adv.service_uuids],
        timeout=SCAN_TIMEOUT,
    )

    if device:
        print(f"[PY] Found by service UUID: {device.name} [{device.address}]")
        return device

    print("[PY] Fallback scan by name...")
    devices = await BleakScanner.discover(timeout=SCAN_TIMEOUT)
    for d in devices:
        print(f"[PY] Seen device: name={d.name}, address={d.address}")
        if d.name == DEVICE_NAME:
            print(f"[PY] Found by name: {d.name} [{d.address}]")
            return d

    return None


async def send_command(client: BleakClient, cmd: str) -> str:
    print(f"[PY] Sending command: {cmd}")
    await client.write_gatt_char(RX_UUID, cmd.encode("utf-8"), response=True)
    await asyncio.sleep(COMMAND_RESPONSE_DELAY)
    data = await client.read_gatt_char(TX_UUID)
    resp = data.decode("utf-8", errors="replace")
    print("[PY] Device response:")
    print(resp)
    return resp


async def sync_time(client: BleakClient):
    unix_ts = int(time.time())
    cmd = f"TIME_SYNC:{unix_ts}"
    print(f"[PY] Auto time sync, unix={unix_ts}")
    await send_command(client, cmd)


def print_menu():
    print()
    print("===== MENU =====")
    print("1 - Получить текущую статистику")
    print("2 - Сбросить текущую статистику")
    print("3 - Отключиться")
    print("4 - Настройка длины шага")
    print("5 - Включить/Выключить дебаг в консоль")
    print("q - Выход")
    print("================")


async def main():
    device = await find_device()
    if device is None:
        print("[PY] Device not found.")
        return

    print(f"[PY] Connecting to {device.address} ...")
    async with BleakClient(device) as client:
        print("[PY] Connected.")

        # Синхронизируем время, чтобы на устройстве были реальные почасовые бакеты
        await sync_time(client)

        while True:
            print_menu()
            choice = input("Выбор: ").strip().lower()

            if choice == "1":
                await send_command(client, "GET_STATS")

            elif choice == "2":
                await send_command(client, "RESET_STATS")

            elif choice == "3":
                await send_command(client, "DISCONNECT")
                print("[PY] Requested disconnect. Exiting client.")
                break

            elif choice == "4":
                await send_command(client, "DEBUG_CHANGE")

            elif choice in ("q", "quit", "exit"):
                print("[PY] Exit without explicit DISCONNECT command.")
                break

            else:
                print("[PY] Unknown command.")

    print("[PY] Client finished.")


if __name__ == "__main__":
    asyncio.run(main())