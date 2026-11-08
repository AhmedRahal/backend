"""Headless radar stream test (no Flask dashboard)."""
import os
import time

from process import get_latest_result, set_error, update_latest_result


def build_source():
    source = os.getenv("RADAR_SOURCE", "sim").lower()
    if source == "sim":
        from sources.simulate_radar import generate_measurements
        return generate_measurements()
    if source == "serial":
        from sources.arduino import read_arduino
        return read_arduino()
    raise ValueError(f"Unknown RADAR_SOURCE: {source!r}")


def main():
    print(f"Starting headless radar test: {os.getenv('RADAR_SOURCE', 'sim')}")
    while True:
        try:
            for measurement in build_source():
                update_latest_result(measurement)
                state = get_latest_result()
                latest = state["last_measurement"] or {}
                print(
                    f"\rangle={latest.get('angle_deg')}°  "
                    f"distance={latest.get('distance_cm')}cm  "
                    f"tracks={len(state['tracked_objects'])}",
                    end="",
                    flush=True,
                )
        except Exception as exc:  # noqa: BLE001
            print(f"\nRadar error: {exc}; retrying in 2s")
            set_error(str(exc))
            time.sleep(2)


if __name__ == "__main__":
    main()
