#!/usr/bin/env python3
"""
Wireless Environment Logger - Serial Logger

Reads CSV data from ESP32 via serial and saves to timestamped CSV file.
Prepends ISO timestamp to each data line.

Usage:
    python logger.py --port /dev/cu.usbserial-XXX --name location1
"""

import argparse
import sys
from datetime import datetime
from pathlib import Path

from serial import Serial, SerialException


def create_filename(name: str) -> str:
    """Generate filename with location name and current timestamp."""
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    return f"{name}_{timestamp}.csv"


def main():
    parser = argparse.ArgumentParser(
        description="Log wireless environment data from ESP32"
    )
    parser.add_argument(
        "--port", "-p",
        required=True,
        help="Serial port (e.g., /dev/cu.usbserial-XXX or COM3)"
    )
    parser.add_argument(
        "--name", "-n",
        required=True,
        help="Location/device identifier for filename"
    )
    parser.add_argument(
        "--baud", "-b",
        type=int,
        default=115200,
        help="Baud rate (default: 115200)"
    )
    parser.add_argument(
        "--output-dir", "-o",
        type=str,
        default=".",
        help="Output directory for CSV files (default: current directory)"
    )

    args = parser.parse_args()

    # Create output directory if needed
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # Generate output filename
    filename = create_filename(args.name)
    filepath = output_dir / filename

    print(f"Opening serial port {args.port} at {args.baud} baud...")

    try:
        ser = Serial(args.port, args.baud, timeout=1)
    except SerialException as e:
        print(f"Error opening serial port: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"Logging to: {filepath}")
    print("Press Ctrl+C to stop\n")

    header_written = False

    try:
        with open(filepath, "w") as f:
            while True:
                try:
                    line = ser.readline().decode("utf-8", errors="replace").strip()
                except SerialException as e:
                    print(f"Serial error: {e}", file=sys.stderr)
                    break

                if not line:
                    continue

                # Check if this is the header line from ESP32
                if line.startswith("timestamp_ms,"):
                    if not header_written:
                        # Write header with prepended ISO timestamp column
                        output_line = f"iso_timestamp,{line}"
                        f.write(output_line + "\n")
                        f.flush()
                        print(f"[HEADER] {output_line}")
                        header_written = True
                    continue

                # Prepend ISO timestamp to data lines
                iso_time = datetime.now().isoformat()
                output_line = f"{iso_time},{line}"

                # Write to file
                f.write(output_line + "\n")
                f.flush()

                # Print to console
                print(output_line)

    except KeyboardInterrupt:
        print("\n\nStopping logger...")
    finally:
        ser.close()
        print(f"Data saved to: {filepath}")


if __name__ == "__main__":
    main()
